/* Userspace TOY utility for DS1302 serial Time of Year RTC. on RPi
 */

const char *const version = "0.013";

/* Copyright (c) 2015 Timothe Litt litt at acm ddot org
 * All rights reserved.
 *
 * This software is provided under GPL V2, including its disclaimer of
 * warranty.  Licensing under other terms may be available from the author.
 *
 * See the COPYING file for the well-known text of GPL V2.
 *
 * Bug reports, fixes, suggestions and improvements are welcome.
 *
 */

/* At this writing, boards with the DS1302 are available for about $2 USD,
 * including battery. They interface to the RPi via 3 GPIO pins, which can
 * be selected for local convenience (and to avoid other projects).
 *
 * This program provides access to all of the chips features, including
 * settng the TOY's time manually or from system time, setting system
 * time from the TOY, accessing the battery backed-up RAM and other
 * functions.  Note that although the user interface is run in local
 * time, the chip is operated using UTC.  
 *
 * The TOY includes a trickle charger for supercapacitors and some
 * rechargeable batteries.  Study the hardware documentation before
 * enabling it.  The charger is NOT suitable for battery types that
 * require temperature monitoring, including NiCADs.  See the Maxim
 * application notes.  Do not enable the charger without adequate
 * hardware knowledge.  Misapplication can cause catastrophic failures.
 *
 * The associated Makefile will install this program on the standard
 * Debian (Raspbian) distribution so that it runs at startup and shutdown,
 * with hourly syncs to system time.
 */

/* Prototypes */
#define _XOPEN_SOURCE 600
#define _BSD_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <libgen.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysexits.h>
#include <syslog.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <utime.h>

#include "rtc-ctl.h"

#if CHECK_NTP && HAVE_SYS_TIMEX
# include <sys/timex.h>
#endif

/* *** Default hardware config *** */

#ifndef CE_PIN
#  define CE_PIN GPIO_P1_16 /* GPIO 23 */
#endif
#ifndef CK_PIN
#  define CK_PIN GPIO_P1_15 /* GPIO 22 */
#endif
#ifndef IO_PIN
#  define IO_PIN GPIO_P1_22 /* GPIO 25 */
#endif

/* *** End of hardware config */

/* Switched from macros to inline functons:
 * a) some of the arguments are messy; not clear they were optimized
 * b) Future chip may deal with century (again), so 3 digits might be useful.
 */

/* 2-digit BCD from binary */

#if 0 /* BCD macro */
#  define BCD(n) ((unsigned char) ( (((n)/10) << 4) | ((n) %10) ))
#else
  static __inline__ unsigned int BCD( unsigned int n ) __attribute__((always_inline));
  static __inline__ unsigned int BCD( unsigned int n )  {
      return ((n / 10 ) << 4) | (n % 10);
      /*    return ((n / 100 ) << 8) | (((n / 10 ) % 10) << 4) | (n % 10); */
  }
#endif

/* Binary from 2-digit BCD */
#if 0 /* BIN macro */
#  define BIN(bcd) ( ((((bcd) & 0xF0) >> 4) * 10) + ((bcd) & 0x0F) )
#else
  static __inline__ unsigned int BIN( unsigned int bcd ) __attribute__((always_inline));
  static __inline__ unsigned int BIN( unsigned int bcd ) {
      return (((bcd >> 4) & 0x0f) * 10) + (bcd & 0x0f);
      /*    return (((bcd >> 8) & 0x0f) * 100) + (((bcd >> 4) & 0x0f) * 10) + (bcd & 0x0f); */
  }
#endif

/* See the DS1302 data sheet, currently: http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
 */

/* Offset of register from clock base */

#define OFFSET(reg) ( ( ((reg) - TOY_SEC) >> 1 ) & 0x3F )

/* DS1302 Register addresses  & bits
 *
 * General command format:
 *
 * +---+---------+---+-----+----+----+----+--------+
 * | 7 |    6    | 5 |  4  |  3 |  2 |  1 |    0   |
 * +---+---------+---+-----+----+----+----+--------+
 * | 1 | RAM/!CK | A4 | A3 | A2 | A1 | A0 | RD/!WR |
 * +---+---------+---+-----+----+----+----+--------+
 *
 * Commands and data are shifted LSB first.
 */

/* Or TOY_READ into address for read */

#define TOY_READ 0x01

/* Read burst to scattered bytes
 * Write burst from an array
 */
#define TOY_SCATTER 0x100
#define TOY_GATHER  0x100

/* Number of clock registers that can be read by a burst */

#define TOY_CLK_REGS (1 + OFFSET(TOY_CTL) - OFFSET(TOY_SEC))

#define TOY_SEC 0x80    /* Write resets count chain to second boundary */
# define TOY_M_HALT 0x80
# define TOY_M_SEC  0x7F

#define TOY_MIN 0x82
# define TOY_M_MIN  0x7F

#define TOY_HR  0x84
# define TOY_M_12HR 0x80
# define TOY_M_PM   0x20
# define TOY_M_HR24 0x3F
# define TOY_M_HR12 0x1F

#define TOY_DAY 0x86
#define TOY_MON 0x88
# define TOY_M_MON_MBZ 0xE0 /* Specified as zero, if TOY not present will float to 1 */
#define TOY_WDY 0x8A
#define TOY_YR  0x8C

#define TOY_CTL 0x8E
# define TOY_M_WP 0x80

/* Trickle charger control */

#define TOY_TCS 0x90
# define TOY_M_TCS_DISABLE 0x5C
# define TOY_M_TCS_ENABLE  0xA0
# define TOY_M_TCS_1D_2K   0xA5 /* 1 Diode, 2K from VCC2 to VCC1 */
# define TOY_M_TCS_1D_4K   0xA6
# define TOY_M_TCS_1D_8K   0xA7
# define TOY_M_TCS_2D_2K   0xA9
# define TOY_M_TCS_2D_4K   0xAA
# define TOY_M_TCS_2D_8K   0xAB

static const struct cmd {
    const char *const string;
    const unsigned char data;
} tcscmds[] = {
    { "disable", TOY_M_TCS_DISABLE },
    { "1d2k", TOY_M_TCS_1D_2K },
    { "1d4k", TOY_M_TCS_1D_4K },
    { "1d8k", TOY_M_TCS_1D_8K },
    { "2d2k", TOY_M_TCS_2D_2K },
    { "2d4k", TOY_M_TCS_2D_4K },
    { "2d8k", TOY_M_TCS_2D_8K },
    { NULL, TOY_M_TCS_DISABLE }
};

/* Clock burst register: atomic read/write of 8 */

#define TOY_CBURST 0xBE

/* On-board RAM */

#define TOY_RAM_BASE 0xC0
#define TOY_RAM_END  0xFC
#define TOY_RAM_REGS (1 + OFFSET(TOY_RAM_END) - OFFSET(TOY_RAM_BASE))

/* RAM burst (1-31 bytes) */

#define TOY_RBURST 0xFE

/* Maximum number of times to read TOY looking for seconds to change.
 * When instrumented, values up to 6300 have been seen, and one
 * in excess of 10,000 (the initial value) timed out.
 */

#define TOY_MAX_READS 30000

/* Command functions */

#define F_READ    0
#define F_SET     1
#define F_SET2SYS 2
#define F_UPDATE  3
#define F_READRAM 4
#define F_SETRAM  5
#define F_READTCS 6
#define F_SETTCS  7
#define F_HALT    8
#define F_TESTRAM 9

/* RTC drift calibration.
 * Time (secs) is at least 9 digits + nsec = 18 digits.
 * double is marginal, so long double is used for math.
 */
struct calib {
    long double lastset;
    long double driftrate;
    int valid;
};

/* Default minimum runtime to update calibration constant.
 * Longer is better - 28 days seems to be a good choice.
 *
 * Two days at 20 PPM is ~ 3.5 secs accumulated error.
 *
 * 1,000,000 seconds is 11.6 days. The 1 sec precision of the TOY
 * (even allowing for the attempts to synchronize reads), amounts
 * to a reasonable 1PPM.  They TOY is sensitive to temperature swings,
 * which can come from the environment or even system load.
 * This is a long enough baseline to cover them as well.
 */

#ifndef MIN_CAL_RUNDAYS
#define MIN_CAL_RUNDAYS 12
#endif

static void printConfig( void );
static void parseSET( int function, char **argv, struct tm *newtime );
static int trytime( const char *dt, const char *fmt, struct tm *newtime );
static char *parseDATE( const char *string );
static void setTOY( struct tm *newtime );
static void readUpdate( int function );
static time_t decodeTOY( unsigned char *clkregs );
static void readCalib( struct calib *calib );
static void writeCalib( struct calib *calib, struct timespec *ts );

static char *formatTime( const struct tm *tm, int has_nsec, long nsec );
static void testTOYram( void );
static void testTOYramPattern( size_t tnum, unsigned char *pattern );

static void dumpData( const unsigned char *data, int n );
static void printTOYregisters( const unsigned char *clkregs, const char *desc );
static void unlockTOY( void );
static void writeTOY( unsigned short reg, ... );
static void readTOY( unsigned short reg, ... );
static void usage( void );

/* Hardware access */

static GPIO_REGS  *gpio;
static TIMER_REGS *timer;

static void initIO(void);
static __inline__  __attribute__((always_inline)) uint64_t read_timer( void );
static void usdelay( uint32_t usecs );

static void gpio_fsel( uint8_t pin, uint8_t mode );
static void gpio_pud( uint8_t pin, uint8_t fcn );

/* Command line */

static char *argv0;
static char *datearg;
static int debug = 0;
static int quiet = 0;
static int testmode = 0;
static const char *calibfilename = CALIB_FILENAME;
#if CHECK_NTP
static int checkntp = 1;
#endif
static int hour12 = 0;
static unsigned long int calrundays = MIN_CAL_RUNDAYS;

/* Includes a subset of hwclock's useful options for compatibility with startup scripts. */

static const struct option options[] = {
    /* hwclock syntax: actions */

    { "hctosys", no_argument, NULL, 's' },        /* Set system from RTC */
    { "set", no_argument, NULL, 'W' },            /* Set RTC to --date */
    { "show", no_argument, NULL, 'r' },           /* Show RTC time */
    { "systohc", no_argument, NULL, 'w' },        /* Set RTC to system time */

    /* hwclock syntax: options */

    { "date", required_argument, NULL, 4 },
    { "utc", no_argument, NULL, 'u' },            /* parse: RTC always in UTC; --localtime is an error */

    /* Clock actions */

    { "read-clock", no_argument, NULL, 'r' },
    { "set-clock", no_argument, NULL, 'W' },
    { "stop-clock", no_argument, NULL, 'Z' },
    { "update-time", no_argument, NULL, 's' },

    /* Clock options */

    { "12-hour-mode", no_argument, &hour12, '1' },
    { "noadjfile", no_argument, NULL, '2' },
    { "adjfile", required_argument, NULL, '@' },

    /* RAM and charger actions */

    { "read-ram", no_argument, NULL, 'R' },
    { "set-ram", no_argument, NULL, 'S' },
    { "read-tricklecharger", no_argument, NULL, 't' },
    { "set-tricklecharger", no_argument, NULL, 'T' },
    { "test-ram", no_argument, NULL, 'X' },

    /* Program commands and options */

    { "caldays", required_argument, NULL, 'm' },
    { "debug", no_argument, &debug, 1 },
    { "quiet", no_argument, &quiet, 1 },
    { "help", no_argument, NULL, 'h' },
#if CHECK_NTP
    { "force", no_argument, &checkntp, 0 },
#endif
    { "show-config", no_argument, NULL, 'c' },
    { "test-mode", no_argument, &testmode, 1 },
    { "version", no_argument, NULL, 'v' },

    /* hwclock syntax that's not supported */

    { "adjust", no_argument, NULL, 'Y' },
    { "getepoch", no_argument, NULL, 'Y' },
    { "setepoch", no_argument, NULL, 'Y' },
    { "epoch", required_argument, NULL, 'Y' },
    { "localtime", no_argument, NULL, 'Y' },
    { "directisa", no_argument, NULL, 'Y' },
    { "badyear", no_argument, NULL, 'Y' },
    { "srm", no_argument, NULL, 'Y' },
    { "arc", no_argument, NULL, 'Y' },
    { "jensen", no_argument, NULL, 'Y' },
    { "funky-toy", no_argument, NULL, 'Y' },

    { NULL, 0, NULL, 0 }
};

int main( int argc, char **argv ) {
    int function = -1;
    int option;
    int optindex;
    struct tm newtime;
    unsigned char ramaddr = 0xFF, ramdata[TOY_RAM_REGS];
    static const struct {
	const char opt;
	const char function;
    } *op, opts[] = {
	{ 'r', F_READ },
	{ 'W', F_SET },
	{ 'w', F_SET2SYS },
	{ 's', F_UPDATE },
	{ 'Z', F_HALT },
	{ 'R', F_READRAM },
	{ 'S', F_SETRAM },
	{ 't', F_READTCS },
	{ 'T', F_SETTCS },
	{ 'X', F_TESTRAM },
	{ '\0', -1 }
    };

    argv0 = strdup( argv[0] );

    memset( ramdata, 0, sizeof(ramdata) );

    while( (option = getopt_long( argc, argv, "rswfWDue1ZRStTdqhcvAJF",
				  options, &optindex )) != -1 ) {
	for( op = opts; op->opt; op++ ) {
	    if( op->opt == option ) {
		if( function >= 0 ) {
		    fprintf( stderr, "Conflicting functions requested\n" );
		    exit( EX_USAGE );
		}
		function = op->function;
		option = 0;
		break;
	    }
	}
	switch( option ) {
	case 'u': /* -u --utc: ignored, since RTC is ALWAYS in UTC */
	    break;

	case 0: /* Flag set by getopt_long, functions */
	    break;

	    /* Short options and long that don't set flags */

	case 'c':
	    printConfig();
	    exit( EX_OK );

	case '1':
	    hour12 = 1;
	    break;

	case '2':
	    calibfilename = NULL;
	    break;

	case '@':
	    calibfilename = optarg;
	    break;


	case 'm': {
	    char *ep;

	    errno = 0;
	    calrundays = strtoul( optarg, &ep, 10 );
	    if( errno || !(*optarg != '\0' && *ep == '\0') || calrundays < 1 ) {
		fprintf( stderr, "Invalid --caldays value\n" );
		exit( EX_USAGE );
	    }
	    break;
	}

	case 'd':
	case 'D':
	    debug = 1;
	    break;

#if CHECK_NTP
	case 'f':
	    checkntp = 0;
	    break;
#endif

	case 4:
	    datearg = optarg;
	    break;

	case 'h':
	    usage();
	    exit( EX_OK );

	case 'v':
	    printf( "rtc-ctl-%s\n", version );
#ifdef __TIMESTAMP__
	    if( debug )
		printf( "rtc-ctl.h: %s\n"
			"rtc-ctl.c: %s\n", rtc_ctl_h_version, __TIMESTAMP__ );
#endif
	    exit( EX_OK );

	case 'q':
	    quiet = 1;
	    break;

	case 'Y':
	case 'A':
	case 'J':
	case 'F':
	    printf( "Command includes an option provided by hwclock, but not supported by rtc-ctl\n" );
	    exit( EX_USAGE );

	case '?':
	default:
	    exit( EX_USAGE );
	}
    }

    if( function < 0 )
	function = F_READ;

    /* Parse non-option arguments */

    switch( function ) {
    case F_SET:
    case F_SET2SYS:
	parseSET( function, argv, &newtime );
	break;

    case F_SETRAM:
	if( argv[optind] && argv[optind+1] ) {
	    unsigned int r;
	    char *ep;
	    r = strtoul( argv[optind], &ep, 16 );
	    if( argv[optind][0] && !*ep && r < TOY_RAM_REGS ) {
		ramaddr = r;
	    } else {
		fprintf( stderr, "Invalid RAM address '%s'\n", argv[optind] );
		exit( EX_USAGE );
	    }

	    r = strtoul( argv[++optind], &ep, 16 );
	    if( argv[optind++][0] && !*ep && r <= 0xFF ) {
		ramdata[0] = r;
	    } else {
		fprintf( stderr, "Invalid RAM data '%s'\n", argv[optind-1] );
		exit( EX_USAGE );
	    }
	} else {
	    fprintf( stderr, "Address and data not specified for -S\n" );
	    exit( EX_USAGE );
	}
	break;

    case F_SETTCS:
	if( argv[optind] ) {
	    const struct cmd *cp = tcscmds;
	    while( cp->string && strcmp( cp->string, argv[optind] ) )
		cp++;
	    if( cp-> string )
		ramdata[0] = cp->data;
	    else {
		fprintf( stderr, "Invalid mode '%s' for -T\n", argv[optind] );
		exit( EX_USAGE );
	    }
	    optind++;
	    break;
	} else {
	    fprintf( stderr, "Mode not specified for -T\n" );
	    exit( EX_USAGE );
	}

    default:
	break;
    }

    if( argv[optind] ) {
	printf( "Extra argument(s) starting with %s\n", argv[optind] );
	exit( EX_USAGE );
    }

    /* Setup hardware to execute commmand */

    initIO();

    /* The DS1302 has internal pulldowns on all 3 IO pins */

    /* Clear CE, which will tri-state IO_PIN in all cases */

    gpio_fsel(CE_PIN, GPIO_FSEL_OUTPUT);
    gpio_pud( CE_PIN, GPIO_PUD_DISABLE );
    GPIO_CLR(CE_PIN);

    /* If CE were high, this could drive IO_PIN if a read were active.
     * Clearing CE first prevents any conflict.
     */

    gpio_fsel(CK_PIN, GPIO_FSEL_OUTPUT);
    gpio_pud( CK_PIN, GPIO_PUD_DISABLE );
    GPIO_CLR(CK_PIN);

    /* Set IO_PIN to output to avoid noise and because all sequences
     * start with driving a command to the TOY.
     */

    gpio_fsel(IO_PIN, GPIO_FSEL_OUTPUT);
    /* gpio_pud( IO_PIN,GPIO_PUD_DOWN ); */
    gpio_pud( IO_PIN, GPIO_PUD_DISABLE );
    GPIO_CLR(IO_PIN);

    /* Execute selected function */

    switch( function ) {
    case F_HALT:
	unlockTOY();
	writeTOY( TOY_SEC, TOY_M_HALT );
	writeTOY( TOY_CTL, TOY_M_WP );
	if( !quiet )
	    printf( "Clock halted\n" );
	break;

    case F_SET2SYS:
    case F_SET:
	setTOY( &newtime );

	/* Fall thru to read back for verification */

    case F_READ:
    case F_UPDATE:
	readUpdate( function );
	break;

    case F_READRAM:
	readTOY( TOY_RBURST, TOY_RAM_REGS, ramdata );
	dumpData( ramdata, TOY_RAM_REGS );
	break;

    case F_SETRAM:
	unlockTOY();
	writeTOY( TOY_RAM_BASE + (ramaddr << 1), ramdata[0] );
	writeTOY( TOY_CTL, TOY_M_WP );
	break;

    case F_READTCS: {
	const struct cmd *cp = tcscmds;

	readTOY( TOY_TCS, ramdata );
	while( cp->string && cp->data != ramdata[0] )
	    cp++;
	if( cp->string )
	    printf( "TCS: %02x = %s\n", ramdata[0], cp->string );
	else
	    printf( "TCS: %02x = unspecified (disabled)\n", ramdata[0] );
	break;
    }

    case F_SETTCS:
	unlockTOY();
	writeTOY( TOY_TCS, ramdata[0] );
	writeTOY( TOY_CTL, TOY_M_WP );
	break;

    case F_TESTRAM:
	testTOYram();
	break;

    default:
	exit( EX_SOFTWARE );
    }

    exit( EX_OK );
}

/* Display configuration
 */

static void printConfig( void ) {
    size_t len;
#define xx(s) xp(s)
#define xp(s) #s
    static const struct {
	const char *const name;
	const unsigned char num;
	const char *const sym;
    } *pin, pins[] = {
	{ "CE/RST", CE_PIN, xx(CE_PIN) },
	{ "CK",     CK_PIN, xx(CK_PIN) },
	{ "IO",     IO_PIN, xx(IO_PIN) },
	{ NULL, 0, NULL }
    };
#undef xp
#undef xx
    len = strlen("Symbol");
    for( pin = pins; pin->name; pin++ ) {
	size_t nl;
	nl = strlen( pin->sym );
	if( nl > len )
	    len = nl;
    }
    printf( "GPIO configuration:\n"
	    "     Name   #  Symbol\n"
	    "    ------ --- " );
    while( len-- )
	fputc( '-', stdout );
    printf( "\n" );
    for( pin = pins; pin->name; pin++ )
	printf( "    %-6s %3u %s\n", pin->name, pin->num, pin-> sym );

    return;
}

/* parse SET command
 */

static void parseSET( int function, char **argv, struct tm *newtime ) {
    time_t t;

    if( function == F_SET ) {
	char *dt = NULL;

	if( datearg ) { /* --date */
	    dt = parseDATE( datearg );
	} else if( argv[optind] && argv[optind+1] ) {
	    dt = malloc( strlen( argv[optind] ) +1 + strlen( argv[optind+1] ) + 1 );
	    if( !dt ) {
		perror( "" );
		exit( EX_USAGE );
	    }
	    strcpy( dt, argv[optind++] );
	    strcat( dt, " " );
	    strcat( dt, argv[optind++] );
	}
	if( dt ) {
	    if( !( trytime( dt, "%d-%b-%Y %T", newtime ) ||
		   trytime( dt, "%m/%d/%Y %T", newtime ) ||
		   trytime( dt, "%Y-%m-%d %T", newtime )
		   ) ) {
		fprintf( stderr, "Unrecognized time argument: %s\n", dt );
		exit( EX_USAGE );
	    }

	    t = mktime( newtime );
	    if( t == -1) {
		fprintf( stderr, "Invalid time\n" );
		exit( EX_USAGE );
	    }
	    if( !quiet ) {
		const struct tm *it;

		it = localtime( &t );
		printf( "Input  time is %s\n", formatTime( it, 0, 0 ) );
	    }
	    free( dt );
	} else {
	    fprintf( stderr, "Set requires --date or date time\n" );
	    exit( EX_USAGE );
	}
    } else if( argv[optind] ) { /* Set from system time. hwclock ignores --date  */
	fprintf( stderr, "Set from system time does not accept arguments\n" );
	exit( EX_USAGE );
    } else {
	time_t t0;

#if CHECK_NTP
	if( checkntp ) {
#  if HAVE_SYS_TIMEX
	    struct timex tx;
	    int ntpstate;

	    memset( &tx, 0, sizeof( tx ) );
	    ntpstate = adjtimex( &tx );
	    if( ntpstate < 0 ) {
		perror( "adjtimed" );
		if( errno == EPERM )
		    exit( EX_NOPERM );
		exit( EX_OSERR );
	    }
	    if( ntpstate == TIME_OK  || /* Sync'd */
		ntpstate == TIME_INS || /* Insert leap */
		ntpstate == TIME_DEL || /* Delete leap */
		ntpstate == TIME_OOP || /* Leap in progress */
		ntpstate == TIME_WAIT ) { /* Leap occurred */
		if( debug )
		    printf( "Kernel clock is synchronized, setting TOY to system time\n" );
	    } else {
		if( !quiet )
		    fprintf( stderr, "Kernel clock is not synchronized, won't set TOY without --force\n" );
		exit( EX_CONFIG );
	    }
#  else
	    FILE *ntpq;
	    int c, ntpOK = 0;

	    ntpq = popen( "2>&1 ntpq -pn", "r" );
	    if( ntpq == NULL ) {
		perror( "ntpq failed. install NTP or use --force" );
		exit( EX_OSERR );
	    }
	    while( (c = fgetc( ntpq )) != EOF ) {
		if( c == 'o' || c == '*' ) { /* PPS peer or selected peer */
		    ntpOK = 1;
		    if( debug )
			printf( "NTP synchronized: %c", c );
		    else
			break;
		}
		/* Flush rest of line (display selected for debug) */
		while( (c = fgetc( ntpq )) != EOF && c != '\n' )
		    if( debug && ntpOK )
			fputc( c, stdout );
		if( debug && ntpOK ) {
		    fputc( '\n', stdout );
		    break;
		}
	    }
	    if( (c = pclose( ntpq )) == -1 ) {
		perror( "ntpq" );
		exit( EX_OSERR );
	    }
	    if( ntpOK && c == 0 ) {
		if( debug )
		    printf( "NTP is active and synchronized, setting TOY to system time\n" );
	    } else {
		if( !quiet )
		    fprintf( stderr, "NTP is not synchronized, won't set TOY without --force\n" );
		exit( EX_CONFIG );
	    }
#  endif
	}
#endif /* CHECK_NTP */
	t0 = time( &t0 );
	if( t0 == -1 ) {
	    perror( "System time - 2038?" );
	    exit( EX_OSERR );
	}
	/* Spin till second changes.  The TOY resets to even second on write, so
	 * this will get it as close as we can to system time.  The TOY will be
	 * behind by the time it takes to shift out the commands + any preemption.
	 * Measured on a 698 BogoMIPS machine as ~400 usec.
	 */
	do {
	    t = time( &t );
	    if( t == -1 ) {
		perror( "System time - 2038?" );
		exit( EX_OSERR );
	    }
	} while( t == t0 );
    }

    if( !gmtime_r( &t, newtime ) ) {
	fprintf( stderr, "Can't convert to UTC, 2038?\n" );
	exit( EX_OSERR );
    }
    return;
}

/* Attempt to parse a dt string with a strptime format.
 */

static int trytime( const char *dt, const char *fmt, struct tm *newtime ) {
    char *p;

    memset( newtime, 0, sizeof(*newtime) );
    p = strptime( dt, fmt, newtime );
    if( !p || *p ) {
	return 0;
    }
    /* Note that these are hardware limits; The TOY probably doesn't handle 2100
     * due to the 400 year leap-year rule.  In any case, these come from the data sheet.
     */
    if( newtime->tm_year + 1900 < 2000 || newtime->tm_year + 1900 >= 2100 ) {
	fprintf( stderr, "Year %u is out of range for TOY\n", newtime->tm_year + 1900 );
	exit( EX_USAGE );
    }
    newtime->tm_isdst = -1;
    return 1;
}

/* Parse --date, compatibly with hwclock.  It's ugly.
 * hwclock runs 'date(1)', so acepts all the expressions that date does.
 * I don't know why it's important to parse --date="next tuesday"...
 */

static char *parseDATE( const char *string ) {
    FILE *date;
#define DC_FMT "date --date=\"%s\" \"+%%d-%%b-%%Y %%T\"\n"
    char *cmd;
    char rsp[ sizeof( "99-Mon-9999 HH:MM:SS\n" ) ];
    size_t len;
    int sts;

    if( strchr( string, '"' ) != NULL ) {
	fprintf( stderr, "--date may not contain '\"'\n" );
	exit( EX_USAGE );
    }

    cmd = malloc( sizeof(  DC_FMT ) + strlen( string ) - sizeof( "%s%%%%" ) + 1 );
    if( !cmd ) {
	perror( "malloc" );
	exit( EX_OSERR );
    }
    sprintf( cmd, DC_FMT, string );
#undef DC_FMT

    date = popen( cmd, "r" );
    if( date == NULL ) {
	perror( "popen" );
	exit( EX_OSERR );
    }
    if( !fgets( rsp, sizeof( rsp ), date ) )
	rsp[0] = '\0';
    if( (sts = pclose( date )) != 0 ) {
	if( sts != -1 ) {
	    if( WIFSIGNALED( sts ) )
		exit( EX_TEMPFAIL );
	    exit( WEXITSTATUS( sts ) );
	}
	perror( "pclose" );
	exit( EX_OSERR );
    }

    free( cmd );

    /* Copy response and remove trailing newline
     */
    len = strlen( rsp );
    cmd = malloc( len + 1 );
    strcpy( cmd, rsp );
    if( len && cmd[--len] == '\n' )
	cmd[len] = '\0';

    return cmd;
}

/* Process set functions
 */

static void setTOY( struct tm *newtime ) {
    time_t newt;
    unsigned char hour, clkregs[TOY_CLK_REGS];
    struct timespec stime, etime;
    struct calib calib;
    long double start;

    /* Record time now so we can account for delays reading TOY
     */

    if( clock_gettime( CLOCK_REALTIME, &stime ) != 0 ) {
	perror( "gettime" );
	exit( EX_OSERR );
    }

    start = ((long double) stime.tv_sec) + (((long double) stime.tv_nsec) / 1e9L);

    /* Compute time_t from newtime & read current calibration
     */

    newt = timegm( newtime );
    readCalib( &calib );

    /* Update calibration if running for at least calrundays.
     */

    if( calib.valid && newt > (calib.lastset + (calrundays * 24 * 60 * 60)) ) {
	readTOY( TOY_CBURST, clkregs );
    
	if( (clkregs[OFFSET(TOY_MON)] & TOY_M_MON_MBZ) != 0 ) {
	    fprintf( stderr, "No TOY detected\n" );
	    if( debug )
		printTOYregisters( clkregs, "as read" );
	    exit( EX_CONFIG );
	}

	/* TOY must be running to calibrate */

	if( !(clkregs[OFFSET(TOY_SEC)] & TOY_M_HALT) ) {
	    unsigned char firstsec;
	    int try = 0;

	    /* Get a stable value - see readUpdate for details.
	     */

	    do {
		firstsec = BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC );
		readTOY( TOY_CBURST, clkregs );
		if( firstsec != BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC ) )
		    break;
		readTOY( TOY_CBURST, clkregs );
	    } while( try++ < TOY_MAX_READS );

	    if( clock_gettime( CLOCK_REALTIME, &etime ) != 0 ) {
		perror( "gettime" );
		exit( EX_OSERR );
	    }
	    if( try < TOY_MAX_READS ) {
		time_t toytime;
		long double error, end;

		/* Compute drift rate from last clock set to now
		 */

		toytime = decodeTOY( clkregs );

		end = ((long double) etime.tv_sec) + (((long double) etime.tv_nsec) / 1e9L);

		error = ((long double) toytime) - end;

		calib.driftrate = error / ( end - calib.lastset );
		if( debug )
		    printTOYregisters( clkregs, "used for calibration" );
	    }
	}
    }

    /* Record time of last set and adjust requested time to compensate for any
     * delays from reading the TOY, debug prints, etc.
     *
     * Note that if calibration wasn't updated for any reason, the last known
     * drift will be used.  If there was no file, this will be zero.
     */
    if( clock_gettime( CLOCK_REALTIME, &etime ) != 0 ) {
	perror( "gettime" );
	exit( EX_OSERR );
    }
    calib.lastset = ((long double) etime.tv_sec) + (((long double) etime.tv_nsec) / 1e9L);
    newt += calib.lastset - start;

    /* Transfer time to TOY
     */

    if( !gmtime_r( &newt, newtime ) ) {
	fprintf( stderr, "Can't convert to UTC, 2038?\n" );
	exit( EX_OSERR );
    }

    if( hour12 )                                          /* 12-hr mode */
	hour = ( newtime->tm_hour ==  0? TOY_M_12HR | BCD( 12 ):
		 newtime->tm_hour == 12? TOY_M_12HR | TOY_M_PM | BCD( 12 ):
		 newtime->tm_hour >  12? TOY_M_12HR | TOY_M_PM | BCD( newtime->tm_hour - 12 ):
		 TOY_M_12HR | BCD( newtime->tm_hour ) );
    else
	hour = BCD( newtime->tm_hour );                    /* 24 hr */

    clkregs[OFFSET(TOY_SEC)] = BCD( newtime->tm_sec );     /* CH is clear */
    clkregs[OFFSET(TOY_MIN)] = BCD( newtime->tm_min );
    clkregs[OFFSET(TOY_HR)]  = hour;
    clkregs[OFFSET(TOY_DAY)] = BCD( newtime->tm_mday );
    clkregs[OFFSET(TOY_MON)] = BCD( newtime->tm_mon +1 );  /* 1-based month */
    clkregs[OFFSET(TOY_WDY)] = BCD( newtime->tm_wday +1 ); /* 1-based, Sun = 1 */
    clkregs[OFFSET(TOY_YR)]  = BCD( (newtime->tm_year + 1900) % 100 );
    clkregs[OFFSET(TOY_CTL)] = TOY_M_WP;

    unlockTOY();                                          /* Clear WP */
    writeTOY( TOY_CBURST | TOY_GATHER, clkregs );

    if( debug )
	printTOYregisters( clkregs, "as written" );

    calib.valid =1;
    writeCalib( &calib, &etime );

    return;
}

/* Process read and update functions
 */

static void readUpdate( int function ) {
    time_t toytime = 0;
    unsigned char firstsec, clkregs[TOY_CLK_REGS];
    struct timespec systime;
    int r;
    struct calib calib;

    memset( &systime, 0, sizeof(systime) );
    readCalib( &calib );

    /* To see stable differences, read the TOY until we see
     * the seconds register change.  That should be close to
     * when it was set, within a few msec. This should help
     * reduce uncertainty in calibration and accuracy checks.
     * "a few" seems to be ~ 5 - 20 msec immediately after --systohc.
     */
    readTOY( TOY_CBURST, clkregs );
    
    if( (clkregs[OFFSET(TOY_MON)] & TOY_M_MON_MBZ) != 0 ) {
	fprintf( stderr, "No TOY detected\n" );
	if( debug )
	    printTOYregisters( clkregs, "as read" );
	exit( EX_CONFIG );
    }
    if( clkregs[OFFSET(TOY_SEC)] & TOY_M_HALT ) {
	fprintf( stderr, "TOY is halted, time is not valid\n" );
	if( debug )
	    printTOYregisters( clkregs, "as read" );
	exit( EX_UNAVAILABLE );
    }

    do {
	firstsec = BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC );
	readTOY( TOY_CBURST, clkregs );
	if( firstsec != BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC ) )
	    break;
	readTOY( TOY_CBURST, clkregs );
    } while( toytime++ < TOY_MAX_READS );

    /* For read, capture system time before any I/O */

    r = clock_gettime( CLOCK_REALTIME, &systime );

    if( debug )
	printTOYregisters( clkregs, "as read" );

    if( toytime >= TOY_MAX_READS ) {
	fprintf( stderr, "TOY does not seem to be running\n" );
	exit( EX_IOERR );
    }

    toytime = decodeTOY( clkregs );

    /* Apply calibration from drift file, if it's available.
     * Stretch elapsed time (sec) by drift rate (sec/sec), rounding to the nearest second.
     * If rate is < 0, TOY runs slower than real time & elapsed is increased to compensate.
     * If rate is > 0, Toy runs faster than real time & elapsed is decreased to compensate.
     */
    if( calib.valid ) {
	long double correction;

	correction = calib.driftrate;

	if( correction != 0 ) {
	    long double elapsed;

	    elapsed = ((long double) toytime) - calib.lastset;

	    if( debug ) {
		printf( "Applying drift correction of %.3Lf PPM to %.3Lf seconds",
			correction * 1e6L, elapsed );
		if( elapsed >= 60. ) {
		    long double dlapsed, rem;

		    dlapsed = elapsed;
		    printf( " (" );
		    if( dlapsed >= 86400. ) {
			rem = modfl( dlapsed / 86400.L, &dlapsed );
			printf( "%.0Lfd ", dlapsed );
			dlapsed = rem * 86400.L;
		    }
		    if( dlapsed >= 3600. ) {
			rem = modfl( dlapsed / 3600.L, &dlapsed );
			printf( "%.0Lfh ", dlapsed );
			dlapsed = rem * 3600.L;
		    }
		    if( dlapsed >= 60. ) {
			rem = modfl( dlapsed / 60.L, &dlapsed );
			printf( "%.0Lfm ", dlapsed );
			dlapsed = rem * 60.L;
		    }
		    printf( "%.0Lfs)", dlapsed );
		}
		printf( " elapsed\n" );
	    }

	    if( correction < 0 )
		toytime = calib.lastset + (elapsed * (1.0L + fabsl( correction ))) + 0.5L;
	    else
		toytime = calib.lastset + (elapsed / (1.0L +        correction  )) + 0.5L;
	}
    }

    if( function == F_UPDATE ) {
	struct timeval settime;
	struct timezone tz;
	struct tm now_tm;
	time_t now;
	int minswest;

	tzset();
	now = time( &now );
	(void) localtime_r( &now, &now_tm );
#if HAVE_TM_GMTOFF
	minswest = -now_tm.tm_gmtoff / 60; /* Seconds east to minutes west */
#else
	minswest = timezone / 60; /* From tzset() / time.h */
	if( now_tm.tm_isdst > 0 )
	    minswest -= 60;
#endif

	memset( &settime, 0, sizeof(settime) );
	settime.tv_sec = toytime;
	settime.tv_usec = 0;

	tz.tz_minuteswest = minswest;
	tz.tz_dsttime = 0;

	/* settimeofday( tv, tz)
	 * First call after bootstrap with tz != NULL: if tv == NULL && tz_minuteswest != 0
	 * the kernel will mark TOY as being on local time.  We never are.  The only reason
	 * to keep the TOY on local time is when it's shared with windows, which for historical
	 * reasons made the mistake of doing so.  This probably doesn't matter since the kernel
	 * doesn't have a driver for this chip.  In any case, code which calls settimeofday
	 * twice, the first time ((struct timeval *)NULL, &struct_timezone ) is triggering
	 * this behavior.
	 */
	if( testmode ) {
	    printf( "DISABLED settimeofday: %ld (%d)\n", (long)settime.tv_sec, tz.tz_minuteswest );
	} else {
	    openlog( basename( argv0 ), LOG_PID, LOG_DAEMON );
	    syslog( LOG_NOTICE, "Settting system time from TOY" ); /* Log old time */

	    if( settimeofday( &settime, &tz ) < 0 ) {
		syslog( LOG_NOTICE, "Settting system time from TOY failed: %s", strerror( errno ) );
		perror( "settimeofday failed" );
		if( errno == EPERM )
		    exit( EX_NOPERM );
		exit( EX_OSERR );
	    }

	    /* Time has been set, get new value as a timespec for display. */

	    r = clock_gettime( CLOCK_REALTIME, &systime );

	    syslog( LOG_NOTICE, "Set system time from TOY" ); /* Log new time */
	    closelog();

	    printf( "Set system time\n" );
	}
    }

    if( function == F_READ || !quiet ) {
	const struct tm *ttime;

	ttime = localtime( &toytime );
	printf( "TOY    time is %s %s\n", formatTime( ttime, -1, 0 ), tzname[ttime->tm_isdst? 1 : 0] );
	if( r != 0 ) {
	    perror( "gettime" );
	    exit( EX_OSERR );
	}
	ttime = localtime( &systime.tv_sec );
	printf( "System time is %s %s\n", formatTime( ttime, 1, systime.tv_nsec ), tzname[ttime->tm_isdst? 1 : 0] );

	if( calib.valid && debug ) {
	    long double tt, elapsed, error;

	    /* Amount of over/under correction */

	    tt = (long double)toytime;
	    elapsed = tt - calib.lastset;
	    error = tt - (systime.tv_sec + (systime.tv_nsec / 1e9));
	    printf( "Remaining offset is %.03Lf sec (%.03Lf PPM)\n", error, (error * 1e6L) / elapsed );
	}
    }
    return;
}

/* Decode UTC TOY registers into a Unix epoch time
 */

static time_t decodeTOY( unsigned char *clkregs ) {
    struct tm newtime;
    time_t toytime;

    memset( &newtime, 0, sizeof(newtime) );
    newtime.tm_sec  = BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC );
    newtime.tm_min  = BIN( clkregs[OFFSET(TOY_MIN)] & TOY_M_MIN );
    if( clkregs[OFFSET(TOY_HR)] & TOY_M_12HR ) {
	newtime.tm_hour = BIN( clkregs[OFFSET(TOY_HR)] & TOY_M_HR12 );
	if( clkregs[OFFSET(TOY_HR)] & TOY_M_PM ) {
	    if( newtime.tm_hour != 12 )
		newtime.tm_hour += 12;
	} else {
	    if( newtime.tm_hour == 12 )
		newtime.tm_hour = 0;
	}
    } else {
	newtime.tm_hour = BIN( clkregs[OFFSET(TOY_HR)] & TOY_M_HR24 );
    }
    newtime.tm_mday = BIN( clkregs[OFFSET(TOY_DAY)] );
    newtime.tm_mon  = BIN( clkregs[OFFSET(TOY_MON)] ) -1;
    newtime.tm_wday = BIN( clkregs[OFFSET(TOY_WDY)] ) -1;
    newtime.tm_year = BIN( clkregs[OFFSET(TOY_YR)] ) + 2000 - 1900;
    if( debug & !(clkregs[OFFSET(TOY_CTL)] & TOY_M_WP) )
	fprintf( stderr, "WP is not set\n" );

    newtime.tm_isdst = 0;
    toytime = timegm( &newtime );
    if( toytime == -1 ) { /* This fails on 32-bit time_t if year is 2038+;
			   * TOY goes to 2099.
			   * Will be fixed when linux time_t expanded.
			   */
	fprintf( stderr, "Failed to convert TOY to UTC, 2038 issue?\n" );
	exit( EX_OSERR );
    }

    return toytime;
}

/* Read calibration file.
 */
static void readCalib( struct calib *calib ) {
    FILE *cfile;
    char line[100];
    char *ep;

    memset( calib, 0, sizeof( *calib ) );

    if( calibfilename == NULL )
	return;

    if( (cfile = fopen( calibfilename, "r" )) == NULL ) {
	perror( calibfilename );
	return;
    }

    if( fgets( line, sizeof( line ), cfile ) == NULL )
	goto reterr;

    errno = 0;
    calib->lastset = strtold( line, &ep );
    if( ep == line || errno != 0 || *ep++ != ' ' || *ep != '(' )
	goto reterr;

    if( fgets( line, sizeof( line ), cfile ) == NULL )
	goto reterr;

    calib->driftrate = strtold( line, &ep );
    if( ep == line || errno != 0 || *ep++ != ' ' || *ep != '(' )
	goto reterr;

    if( fgets( line, sizeof( line ), cfile ) == NULL || strcmp( line, "UTC\n" ) )
	goto reterr;

    calib->valid = 1;

 reterr:
    fclose( cfile );
    return;
}

/* Write calibration file
 */
static void writeCalib( struct calib *calib, struct timespec *ts ) {
    FILE *cfile;
    struct tm tm;
    time_t t;
    char *tmpname;

    if( calibfilename == NULL || !calib->valid )
	return;

    t = ts->tv_sec;
    if( !gmtime_r( &t, &tm ) ) {
	fprintf( stderr, "Can't convert to UTC, 2038?\n" );
	exit( EX_OSERR );
    }

    /* assert( sizeof( ".bak" ) == sizeof( ".tmp" ) )
     */

    tmpname = malloc( strlen( calibfilename ) + sizeof( ".bak" ) );
    if( tmpname == NULL ) {
	perror( "malloc" );
	exit( EX_OSERR );
    }

    /* If a calib file exists, back it up first.
     * Note that one can't simply switch to the saved
     * file, as the TOY has been reset, so the last time
     * set will be wrong.  But one can see if the drift
     * rate is wildly different, and perhaps restore it manually.
     */

    if( (cfile = fopen( calibfilename, "r" )) != NULL ) {
	FILE *bfile;
	struct stat statbuf;
	struct utimbuf utbuf;
	int c;

	if( fstat( fileno( cfile ), &statbuf ) != 0 ) {
	    perror( calibfilename );
	    exit( EX_OSERR );
	}
	utbuf.actime  = statbuf.st_atime;
	utbuf.modtime = statbuf.st_mtime;

	sprintf( tmpname, "%s.bak", calibfilename );
	if( (bfile = fopen( tmpname, "w" )) == NULL ) {
	    perror( tmpname );
	    exit( EX_IOERR );
	}

	while( (c = fgetc( cfile )) != EOF ) {
	    if( fputc( c, bfile ) == EOF ) {
		perror( tmpname );
		exit( EX_IOERR );
	    }
	}

	if( fclose( bfile ) ) {
	    perror( tmpname );
	    exit( EX_IOERR );
	}

	utime( tmpname, &utbuf );

	if( debug )
	    fprintf( stderr, "Previous calibration saved in %s\n", tmpname );

	fclose( cfile );
    }

    /* Write the new file.
     * To minimize the chance of loss in a crash, write to a temporary
     * file, then rename to install and replace any old file.  This is
     * usually atomic.
     */

    sprintf( tmpname, "%s.tmp", calibfilename );

    if( (cfile = fopen( tmpname, "w" )) == NULL ) {
	perror( tmpname );
    } else {
	fprintf( cfile, "%.9Lf (%s UTC)\n"
		        "%.12Lf (%.3Lf PPM)\n"
	                "%s\n",
		 calib->lastset, formatTime( &tm, 1, ts->tv_nsec ),
		 calib->driftrate, calib->driftrate * 1e6L,
		 "UTC" );

	if( fclose( cfile ) ) {
	    perror( tmpname );
	} else {
	    if( rename( tmpname, calibfilename ) == 0 ) {
		if( debug )
		    fprintf( stderr, "Calibration data written in %s\n", calibfilename );
	    } else {
		perror( calibfilename );
	    }
	}
    }

    free( tmpname );

    return;
}

/* Display time with year next to date (before time).
 * Options:
 *  has_nsec == 0: just date and time
 *  has_nsec  < 0: date and time with padding for msec
 *  has_nsec == 1: date and time + milliseconds (nsec / 1,000,000)
 */
static char *formatTime( const struct tm *tm, int has_nsec, long nsec ) {
    static char rstring[125+sizeof(".mmm")]; /* 25 suffices for POSIX/EN locales; allow more */

    rstring[0] = '\0';

    if( !has_nsec ) {
	strftime( rstring, sizeof(rstring), "%a %b %d %Y %H:%M:%S", tm );
	return rstring;
    }
    if( has_nsec < 0 )
	strftime( rstring, sizeof(rstring), "%a %b %d %Y %H:%M:%S    ", tm );
    else {
	size_t n;

	n = strftime( rstring, sizeof(rstring), "%a %b %d %Y %H:%M:%S", tm );
	if( n >= sizeof(rstring) - sizeof( ".mmm" ) )
	    abort();
	sprintf( rstring+n, ".%03.0f", ((double)nsec) / 1e6 ); /* msec (rounded) */
    }

    return rstring;
}

/* Test TOY RAM
 * This is a RAM test, but it's really intended to exercise the interface to verify
 * correct installation. Odds of the RAM being bad are pretty slim..
 */
static void testTOYram( void ) {
    static const unsigned char dataPatterns[] = {
	0x00, 0xFF, 0x55, 0xAA,                         /* 0s, 1s, alternating */
	0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, /* Floating 1 */
	0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE, /* Floating 0 */
    };
    unsigned char pbuf[TOY_RAM_REGS];
    size_t tnum, addr;

    printf( "Testing" );

    for( tnum = 0; tnum < sizeof( dataPatterns ); tnum++ ) {
	memset( pbuf, dataPatterns[tnum], sizeof( pbuf ) );
	testTOYramPattern( tnum, pbuf );
    }

    for( addr = 0; addr < sizeof( pbuf ); addr++ ) {
	pbuf[addr] = sizeof( pbuf ) - addr;
    }
    testTOYramPattern( tnum, pbuf );

    memset( pbuf, 0, sizeof( pbuf ) );
    unlockTOY();
    writeTOY( TOY_RBURST | TOY_GATHER, TOY_RAM_REGS, pbuf );
    writeTOY( TOY_CTL, TOY_M_WP );

    printf( "\n" );
    return;
}

/* Announce, write, read and verify a single pattern */

static void testTOYramPattern( size_t tnum, unsigned char *pattern ) {
    unsigned char rbuf[TOY_RAM_REGS];
    int failed;

    printf( "...%u", (int)tnum );

    unlockTOY();
    writeTOY( TOY_RBURST | TOY_GATHER, TOY_RAM_REGS, pattern );
    writeTOY( TOY_CTL, TOY_M_WP );

    readTOY( TOY_RBURST, TOY_RAM_REGS, rbuf );

    if( (failed = memcmp( pattern, rbuf, sizeof( rbuf ) )) != 0 ) {
	printf( "-Failed" );
    }
    if( failed || debug ) {
	printf( "\nWrote\n" );
	dumpData( pattern, TOY_RAM_REGS );
	printf( "Read\n" );
	dumpData( rbuf, sizeof( rbuf ) );
	if( failed )
	    exit( EX_IOERR );
    }

    printf( "-OK" );

    return;
}

/* print random byte data array
 */

static void dumpData( const unsigned char *data, int n ) {
    int i;

    for( i = 0; i < n; i++ ) {
	if( (i % 8) == 0 ) {
	    if( i )
		printf( "\n" );
	    printf( "%02x:", i );
	}
	printf( " %02x", *data++ );
    }
    printf( "\n" );
    return;
}

/* Print decoded TOY clock registers
 */
static void printTOYregisters( const unsigned char *clkregs, const char *desc ) {
    printf( "TOY Clock registers %s (UTC):\n"
	    "%02X: %02x %3s %02u sec\n",
	    desc,
	    (TOY_SEC + TOY_READ),
	    clkregs[OFFSET(TOY_SEC)],
	    (clkregs[OFFSET(TOY_SEC)] & TOY_M_HALT? "CH": "RUN"),
	    BIN( clkregs[OFFSET(TOY_SEC)] & TOY_M_SEC ) );
    printf( "%02X: %02x     %02u min\n",
	    (TOY_MIN + TOY_READ),
	    clkregs[OFFSET(TOY_MIN)],
	    BIN( clkregs[OFFSET(TOY_MIN)] & TOY_M_MIN ) );
    printf( "%02X: %02x %3s %02u%s hr\n",
	    (TOY_HR + TOY_READ),
	    clkregs[OFFSET(TOY_HR)],
	    (clkregs[OFFSET(TOY_HR)] & TOY_M_12HR? "12H": "24H"),
	    (clkregs[OFFSET(TOY_HR)] & TOY_M_12HR?
	     BIN( clkregs[OFFSET(TOY_HR)] & TOY_M_HR12 ):
	     BIN( clkregs[OFFSET(TOY_HR)] & TOY_M_HR24 )),
	    (clkregs[OFFSET(TOY_HR)] & TOY_M_12HR?
	     clkregs[OFFSET(TOY_HR)] & TOY_M_PM? " PM": " AM": "" ) );
    printf( "%02X: %02x     %02u date\n",
	    (TOY_DAY + TOY_READ),
	    clkregs[OFFSET(TOY_DAY)],
	    BIN( clkregs[OFFSET(TOY_DAY)] ) );
    printf( "%02X: %02x     %02u month\n",
	    (TOY_MON + TOY_READ),
	    clkregs[OFFSET(TOY_MON)],
	    BIN( clkregs[OFFSET(TOY_MON)] ) );
    printf( "%02X: %02x     %02u weekday\n",
	    (TOY_WDY + TOY_READ),
	    clkregs[OFFSET(TOY_WDY)],
	    BIN( clkregs[OFFSET(TOY_WDY)] ) );
    printf( "%02X: %02x     %02u year\n",
	    (TOY_YR + TOY_READ),
	    clkregs[OFFSET(TOY_YR)],
	    BIN( clkregs[OFFSET(TOY_YR)] ) );
    printf( "%02X: %02x %3s ctl\n",
	    (TOY_CTL + TOY_READ),
	    clkregs[OFFSET(TOY_CTL)],
	    clkregs[OFFSET(TOY_CTL)] & TOY_M_WP? "WP": "" );
    return;
}

/* Clear WP in the TOY, checking for a response.
 *
 * This verifies that the TOY is preset for all write operations.
 */

static void unlockTOY( void ) {
    unsigned char ctl;
    unsigned char setval;

    setval = testmode? TOY_M_WP: 0;

    writeTOY( TOY_CTL, setval );
    readTOY( TOY_CTL | TOY_SCATTER, &ctl );

    if( ctl != setval ) {
	fprintf( stderr, "No TOY detected\n" );
	if( debug )
	    fprintf( stderr, "Wrote %02x to CTL, read %02x\n", setval, ctl );
	exit( EX_CONFIG );
    }
    return;
}

/* TOY I/O routines.
 * These are written for correctness, not maximum speed.
 * Some delays are a bit longer than strictly necessary.
 *
 * There is no minimum speed required; the part is specified to
 * work down to DC.  There is a 1sec max between writing TOY_SEC
 * and raising CE to avoid roll-over.  But as we set the clock
 * using BURST, this doesn't apply.
 */

static void writeTOY( unsigned short reg, ... ) {
    va_list ap;
    int i, n;
    unsigned char *dp = NULL;
    unsigned short gather = reg & TOY_GATHER;

    va_start( ap, reg );

    switch( (unsigned char)reg ) {
    case TOY_CBURST:
	n = TOY_CLK_REGS; /* All 8 must be written for transfer to counter to work */
	break;

    case TOY_RBURST:
	n = va_arg(ap, int );
	if( n <= 0 || n > TOY_RAM_REGS ) {
	    fprintf( stderr, "Internal error, %d RAM write\n", n );
	    exit( EX_SOFTWARE );
	}
	break;

    default:
	n = 1;
    }

    if( gather ) /* Write burst from an array */
	dp = va_arg( ap, unsigned char * );

    GPIO_CLR( CK_PIN );
    NB_GPIO_SET( CE_PIN );
    usdelay( 4 ); /* Tcc */

    /* Command/address byte */

    for( i = 0; i < 8; i++ ) {
	NB_GPIO_SET_TO( IO_PIN, reg & 0x01 );
	reg >>= 1;
	usdelay( 2 ); /* Tdc */

	NB_GPIO_SET( CK_PIN );
	usdelay( 2 ); /* Tcdh, Tch */
	NB_GPIO_CLR( CK_PIN );
	usdelay( 2 ); /* Tcl */
    }

    /* (Burst of) data byte(s) */

    while( n-- ) {
	unsigned char data = gather? *dp++: va_arg( ap, int );

	for( i = 0; i < 8; i++ ) {
	    NB_GPIO_SET_TO( IO_PIN, data & 0x01 );
	    data >>= 1;
	    usdelay( 2 ); /* Tdc */

	    NB_GPIO_SET( CK_PIN );
	    usdelay( 2 ); /* Tdch, Tch */
	    NB_GPIO_CLR( CK_PIN );
	    usdelay( 2 ); /* Tcl */
	}
    }

    NB_GPIO_CLR( IO_PIN );
    NB_GPIO_CLR( CE_PIN ); /* This transfers clock data to counting registers */
    usdelay( 4 ); /* Tcwh & MB */

    va_end( ap );
    return;
}

/* Read TOY register(s)
 */
static void readTOY( unsigned short reg, ... ) {
    va_list ap;
    int i, n;
    unsigned char *data = NULL;
    unsigned short scatter = reg & TOY_SCATTER;

    va_start( ap, reg );

    switch( (unsigned char)reg ) {
    case TOY_CBURST:
	n = TOY_CLK_REGS;
	break;

    case TOY_RBURST:
	n = va_arg(ap, int );
	if( n <= 0 || n > TOY_RAM_REGS ) {
	    fprintf( stderr, "Internal error, %d RAM read\n", n );
	    exit( EX_SOFTWARE );
	}
	break;
    default:
	n = 1;
    }
    if( !scatter ) /* Read burst into an array */
	data = va_arg( ap, unsigned char * );

    reg |= TOY_READ;

    GPIO_CLR( CK_PIN );
    NB_GPIO_SET( CE_PIN ); /* Captures counting registers */
    usdelay( 4 ); /* Tcc */

    /* Drive read command to TOY.
     * First bit from TOY is driven on the falling
     * edge of the last command clock.  Don't
     * produce that in this loop so we can switch
     * pin to input first.
     */
    
    for( i = 0; i < 8; i++ ) {
	NB_GPIO_SET_TO( IO_PIN, reg & 0x01 );
	reg >>= 1;
	usdelay( 2 ); /* Tdc */

	NB_GPIO_SET( CK_PIN );
	usdelay( 2 ); /* Tcdh, Tch */

	if( i < 7 ) {
	    NB_GPIO_CLR( CK_PIN );
	    usdelay( 2 ); /* Tcl */
	}
    }

    /* Last command bit has been shifted in,
     * but TOY won't drive until CLK falls.
     * Switch pin to input first.
     */

    gpio_fsel( IO_PIN, GPIO_FSEL_INPUT );
    usdelay( 2 );

    /* Read (burst of) data byte(s) */

    while( n-- ) {
	if( scatter ) /* Read to non-array */
	    data = va_arg( ap, unsigned char * );

	*data = 0;
	for( i = 0; i < 8; i++ ) {
	    NB_GPIO_CLR( CK_PIN );
	    usdelay( 2 ); /* Tcdd, Tcl */

	    *data >>= 1;
	    if( NB_GPIO_IS_SET( IO_PIN ) ) {
		*data |=0x80;
	    }
	    /* Shift the next bit if more in this byte
	     * or if another byte follows in a burst.
	     * Don't shift if last bit. TOY sets IO to HiZ when CK_PIN is high.
	     */
	    if( i < 7 || n ) {
		NB_GPIO_SET( CK_PIN );
		usdelay( 2 ); /* Tch */
	    }
	}
	data++;
    }

    NB_GPIO_CLR( CE_PIN );
    usdelay( 2 );  /* Tcdz */
    gpio_fsel( IO_PIN, GPIO_FSEL_OUTPUT );
    GPIO_CLR( IO_PIN );

    va_end(ap);
    return;
}

static void usage( void ) {
    printf( "TOY manager version %s for DS1302 on RPi\n", version );
    printf( "\n" );
    printf( "Usage: rtc-ctl [options] args\n" );
    printf( "\n" );
    printf( " --quiet                                 --debug\n" );
    printf( "  -q - only report errors                 -d - report details\n" );
    printf( "\n" );
    printf( " --set-clock | --set --date=\"string\"   --read-clock | --show\n" );
    printf( "  -W - set TOY time (args below)          -r - read TOY time (default)\n" );
    printf( "\n" );
    printf( " --systohc                               --adjfile=%s\n", calibfilename );
    printf( "  -w  - set TOY from system time         --noadjfile\n" );
    printf( " --caldays=n\n" );
    printf( "   Minimum clock runtime (days) to update calibration. Default %u, min 1.\n", MIN_CAL_RUNDAYS );
    printf( "   28 or more will usually give better results.\n" );
    printf( "\n" );
#if CHECK_NTP
    printf( " --update-time | --hctosys               --force\n" );
    printf( "  -s - update system time from TOY        -f - set even if system time not syncd\n" );
#else
    printf( " --update-time | --hctosys\n" );
    printf( "  -s - update system time from TOY\n" );
#endif
    printf( "\n" );
    printf( " --12-hour-mode                          --stop-clock\n" );
    printf( "  -1 - set clock in 12-hour (AM/PM) mode  -Z - Halt clock in nanopower mode\n" );
    printf( "\n" );
    printf( " --show-config                           --version\n" );
    printf( "  -c - display GPIO configuration         -v - display version\n" );
    printf( "\n" );
    printf( " --read-ram                              --set-ram\n" );
    printf( "  -R - read battery backed-up RAM         -S - set BBRAM (args below)\n" );
    printf( "\n" );
    printf( " --test-ram                              --test-mode\n" );
    printf( "  -X - test TOY memory & interface         Do everyting except write clock and set time.\n" );
    printf( "\n" );
    printf( " --read-tricklecharger                   --set-tricklecharger\n" );
    printf( "  -t - read charger configuration (TCS)   -T - set charger config (args below)\n" );
    printf( "\n" );
    printf( " The clock commands accept (a subset of) hwclock syntax.  This allows rtc-ctl to\n" );
    printf( " replace hwclock in some startup files.\n" );
    printf( "\n" );
    printf( " -W accepts 2 arguments: date and time\n" );
    printf( "  date can be specified as any one of:\n" );
    printf( "    dd-MMM-yyyy\n" );
    printf( "    mm/dd/yyyy\n" );
    printf( "    yyyy-mm-dd\n" );
    printf( "  time is always hh:mm:ss (24 Hr)\n" );
    printf( "  If neither is specified, the system's time may be copied to the TOY\n" );
#if CHECK_NTP
    printf( "  To prevent setting the TOY to an invalid time, the system time\n" );
    printf( "  is only copied if system time is synchronized.  To copy\n" );
    printf( "  the system time without checking NTP, specify --force (or -f)\n" );
#endif
    printf( "\n" );
    printf( " Time is input and reported as local time, but stored in the TOY as UTC\n" );
    printf( " Nanopower mode is for storage/shipping. Time is lost.\n" );
    printf( " 12-hour mode is supported, but converted to/from system time (24 Hr.)\n" );
    printf( "The TOY will drift with respect to real time, typically 20PPM, or about 1 min/month.\n" );
    printf( "The --set and --systohc commands update the drift file, which allows the \n" );
    printf( "commands that read the clock to compensate.  --noadjfile suppresses both.\n" );
    printf( "Drift is only updated --caldays days (or more) after the clock has been set.\n" );
    printf( "--systohc should be run weekly or monthly to keep drift accumulation reasonable.\n" );
    printf( "\n" );
    printf( "-S requires an address (hex: 00 - 1E) and data (00-FF)\n" );
    printf( "\n" );
    printf( "-T requires one of the following 7 modes:\n" );
    printf( "    disabled\n" );
    printf( "    1d2k    2d2k\n" );
    printf( "    1d4k    2d4k\n" );
    printf( "    1d8k    2d8k\n" );
    printf( "   Do not modify TCS without hardware knowledge; see DS1302\n" );
    printf( "   and battery/supercap datasheets.  Batteries can fail destructively.\n" );

    return;
}

/* Initialize for (memory-mapped GP)IO
 * Locate the IO peripherals IO address'.
 * mmap them to a virtual address
 * setup pointers
 */

static void initIO(void) {
    FILE *proc;
    uint32_t io_base, io_len;
    int gpio_fd, pagesize;
    uint8_t *io;

    /* Device tree data packed in 32-bit words, big-endian */
#define DT_UNPACK( v ) ( ( (v)[0] << 24 ) |  ( (v)[1] << 16 ) |  ( (v)[2] << 8 ) |  (v)[3] )
  
    if( (proc = fopen( "/proc/device-tree/soc/ranges", "rb" )) != NULL ) {
	uint8_t dtv32[4];
	
	if( fseek( proc, 4, SEEK_SET ) != 0 ) {
	    perror( "seek" );
	    exit( EX_IOERR );
	}
	if( fread( dtv32, 1, sizeof( dtv32 ), proc ) != sizeof( dtv32 ) ) {
	    fprintf( stderr, "Device tree read failed\n" );
	    exit( EX_IOERR );
	}
	io_base = DT_UNPACK( dtv32 );

	if( fread( dtv32, 1, sizeof( dtv32 ), proc ) != sizeof( dtv32 ) ) {
	    fprintf( stderr, "Device tree read failed\n" );
	    exit( EX_IOERR );
	}
	io_len = DT_UNPACK( dtv32 );

	fclose( proc );
    } else if( (proc = fopen( "/proc/iomem", "r" )) != NULL ) {
	unsigned long start, end;
	int n = 0;
	char nl = 0;
	char buf[257];

	while( fgets( buf, sizeof(buf), proc ) != NULL ) {
		if( (n = sscanf( buf, "%lx-%lx : bcm2708_gpio%c",
				 &start, &end, &nl )) == 3 && nl != 0 ) {
		    break;
		}
	}
	if( n == 3 && nl == '\n' ) {
	    io_base = start - GPIO_BASE_OFFSET;
	    io_len = end + 1 - start;

	    if( TIMER_BASE_OFFSET > GPIO_BASE_OFFSET ) {
		exit( EX_CONFIG );
	    }
	    io_len += GPIO_BASE_OFFSET;
	} else {
	    fprintf( stderr, "Unable to find IO region\n" );
	    exit( EX_CONFIG );
	}
	fclose( proc );
    } else {
	fprintf( stderr, "No mechanism for finding IO region\n" );
	exit( EX_UNAVAILABLE );
    }

    gpio_fd = open( "/dev/mem", O_RDWR | O_SYNC, 0 );
    if( gpio_fd == -1 ) {
	perror( "Unable to access physical memory" );
	if( errno == EPERM )
	    exit( EX_NOPERM );
	exit( EX_OSERR );
    }

    pagesize = getpagesize();
    io_len = ( (io_len + pagesize -1) / pagesize ) * pagesize;

    io = mmap( NULL, io_len, (PROT_READ|PROT_WRITE), MAP_SHARED, gpio_fd, io_base );
    if( gpio == MAP_FAILED ) {
	perror( "Unable to map IO region" );
	exit( EX_OSERR );
    }

    gpio = (void *)(io + GPIO_BASE_OFFSET);
    timer = (void *)(io + TIMER_BASE_OFFSET);

    return;
}

/* Spinwait timers (the values required are microseconds, so calling the OS isn't worthwhile, or accurate))
 * The timer's counters are configured to count at 1MHz.  32 bits overflow in about 72 mins, so barring
 * some process control event (suspend, debug, etc), the do loop should execute at most twice.
 */

static __inline__  __attribute__((always_inline)) uint64_t read_timer( void ) {
    uint32_t hi1, lo, hi2;
    uint64_t value;

    /* Assumes caller provides MB()s */

    do {
	hi1 = timer->chi;
	lo  = timer->clo;
	hi2 = timer->chi;
    } while( hi1 != hi2 );

    value = hi2;
    value = (value << 32) | lo;

    return value;
}

static void usdelay( uint32_t usecs ) {
    uint64_t end;

    MB();
    end = read_timer() + usecs;

    while( read_timer() < end )
	;
    MB();
    return;
}

/* Select GPIO pin's function
 */

static void gpio_fsel( uint8_t pin, uint8_t mode ) {
    uint8_t regn, bitn;
    uint32_t mask;

    regn = pin / 10;
    bitn = (pin % 10) * 3;
    mask = GPIO_FSEL_FNMASK << bitn;
    MB();
    gpio->fsel[regn] = (gpio->fsel[regn] & ~mask) | (((uint32_t)mode) << bitn);
    MB();

    return;
}

/* Select GPIO pin's pullups
 */

static void gpio_pud( uint8_t pin, uint8_t fcn ) {
    uint8_t regn, bitn;
    uint32_t mask;

    regn = (pin & 0x20)? 1 : 0;
    bitn = pin & 0x1F;
    mask = 1 << bitn;

    /* Value in pud goes to all pins, but is selectively clocked by
     * bit(s) in pudclk
     */

    MB();
    gpio->pud = fcn;
    usdelay( 150 );               /* Specified as cycles, but cycle won't be slower than 1MHz */
    gpio->pudclk[regn] = mask;
    usdelay( 150 );

    gpio->pud = GPIO_PUD_DISABLE; /* Specified, but (a) why before dropping CLK (b) why required? */
    gpio->pudclk[regn] = 0;
    MB();

    return;
}

/* *EOF* */
