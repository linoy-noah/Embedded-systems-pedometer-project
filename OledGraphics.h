#include "Compiler.h"
#include "GenericTypeDefs.h"

#define	oledWR			LATEbits.LATE1
#define	oledWR_TRIS		TRISEbits.TRISE1
#define	oledRD			LATEbits.LATE0
#define	oledRD_TRIS		TRISEbits.TRISE0
#define	oledCS			LATEbits.LATE2
#define	oledCS_TRIS		TRISEbits.TRISE2
#define	oledRESET		LATDbits.LATD1
#define	oledRESET_TRIS	TRISDbits.TRISD1
#define	oledD_C			LATBbits.LATB5
#define	oledD_C_TRIS	TRISBbits.TRISB5


#define	START_RMW	0xE0
#define	STOP_RMW	0xEE

#define	MAX_X	131
#define	MAX_Y	63

typedef enum {
	thin,
	thick,
	fat
} LineWidth ;

void drawLine( BYTE x0, BYTE y0, BYTE x1, BYTE y1, LineWidth lw );
