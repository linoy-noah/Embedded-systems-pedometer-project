#include "OledGraphics.h"
#include "oled.h"

//#ifdef _VISUAL
//#include "VisualSpecials.h"
//#endif // VISUAL

//			====================================================================
//			====================================================================
//				OLED Graphics Package for the PICSTART1 kit
//				Provides routines to set individual pixels and draw lines
//
//				Written by: Amit Resh, Copyright (c) Jan 2015
//			====================================================================
//			====================================================================

//			OLED pixel coordinate system
//
//			0                   X                  131  
//			+-----------------------------------------
//	  	   0|
//			|
//		Y	|
//			|
//			|
//			|
//		  63|


#undef	ReadData				//Microchip wrote a bad macro to Read an OLED byte

static BYTE ReadData(void)				//This replaces that bad macro with a good routine
{
    BYTE data;

	TRISD = 0xFF;				//Make sure PortD is INPUT

    oledRD = 1;					//Set _Read/_Write lines High (inactive)
	oledWR = 1;
    oledD_C	= 1;				//Data (A0=1)
    oledCS = 0;					//Chipselect the OLED controller

    oledRD = 0;					//Dummy Read
	PMPDelay() ;
    oledRD = 1;
	Nop() ;

    oledRD = 0;					//Real Read
	Nop() ;
	data = PORTD ;				//Get data from PORT D
    oledRD = 1;	
    oledCS = 1;					//Deselect OLED chip
}

//
//		Swap (x0,y0) <=> (x1,y1)
//
static void swap( BYTE *x0, BYTE *y0, BYTE *x1, BYTE *y1 )
{
	int t ;

	t = *x0 ;
	*x0 = *x1 ;
	*x1 = t ;

	t = *y0 ;
	*y0 = *y1 ;
	*y1 = t ;
}

//
//	Draw the mask (set bits of mask) at OLED position
//
static void DrawMask( BYTE mask )
{
	BYTE oledByte ;

	WriteCommand(START_RMW) ;				//Start Read-Modify-Write
	oledByte = ReadData() ;					//Get existing byte at OLED location
	oledByte ^= mask ;						//Set mask bits
	WriteData(oledByte) ;					//Write it back to OLED
	WriteCommand(STOP_RMW) ;				//Stop Read-Modify-Write
}


//
//	Set the OLED Page/Col position
//
static void SetOLEDPos( BYTE page, BYTE col)
{
	WriteCommand(0xB0+page);				//Set Page
	col += OFFSET;							//Take into account the fixed OFFSET
	WriteCommand(0x00+(col&0x0F));			//Set Column with 2 commands
	WriteCommand(0x10+((col>>4)&0x0F));
}

//
//	Set a sequence of yCount bits, starting at (x,y)
//		provided that they fit in a single OLED location
//
//	Return: number of bits successfully set
//
static BYTE setYPixels( BYTE x, BYTE y, BYTE yCount, LineWidth lw)
{
	BYTE mask=0, bitCount ;
	BYTE pg = (y>>3) ;						//Calculate Page location
	BYTE bit = y % 8 ;						//Calculate 1st bit in sequence

	for (bitCount=0 ; bit<=7 && yCount ; ++bitCount, --yCount, ++bit )
		mask |= 1<<bit ;					//Set sequence bits in mask, provided they fit at same OLED location

	SetOLEDPos( pg, x ) ;					//Set OLED position
	DrawMask( mask ) ;						//Set all bits in mask

	if (lw > thin && x<MAX_X) {
		SetOLEDPos( pg, x+1 ) ;				//Set same pixels to the right
		DrawMask( mask ) ;
	}
	if (lw > thick && x>0) {
		SetOLEDPos( pg, x-1 ) ;				//Set same pixels to the left
		DrawMask( mask ) ;
	}

	return bitCount ;						//Return: number of bits succeeded
}


//
//	Set bit at OLED coordinate (x,y)
//
static void setXPixel( BYTE x, BYTE y, LineWidth lw)
{
	BYTE dy=1, pix ;								//Default cross-section (along Y axis) is 1 pixel

	if (lw > thin && y>0) {
		--y ;										//If wider line, use longer cross-section
		++dy ;
	}
	if (lw > thick && y+dy < MAX_Y)
		++dy ;										//If yet wider, make even longer cross-section

	for ( ; dy ; dy -= pix, y += pix )
		pix = setYPixels( x, y, dy, thin ) ;		//Set cross-section pixels
}


//
//	Draw a straight line between (x0,y0) ==> (x1,y1)
//
void drawLine( BYTE x0, BYTE y0, BYTE x1, BYTE y1, LineWidth lw )
{
	BYTE x, y, DY, DX, pix ;
	BYTE dx ;
	BYTE dy ;
	int stepX, xLimit ;

	int yTOx100, Dy100, xTOy100, Dx100, q ;					//xxxx100 variables store centi-values. I.e., integers with unit 1/100
	BYTE Err100 ;

	if (y0 > y1)
		swap( &x0, &y0, &x1, &y1 ) ;						//Make Sure y0 ==> y1 is always positive direction

	dy = 1+ (y1-y0) ;										//DY:  y0==>y1
	if (x1>=x0) {											//DX:  abs | x0==>x1 |
		dx = 1+ (x1-x0) ;									//stepX: 1	-	x1>= x0
		stepX = 1 ;											//		-1	-	Otherwise
	}
	else {
		dx = 1+ (x0-x1) ;
		stepX = -1 ;
	}

	if (dy>=dx) {											//Is DY larger than DX?
		Dy100 = (int)dy*100 ;								//Yes. Iterate on X
		yTOx100 = Dy100 /dx ;								//Draw yTox pixels each iteration

		for (x=x0, y=y0, Err100=0, xLimit = (int)x1*stepX ; (int)x*stepX <= xLimit ; x += stepX) {
			DY = (BYTE)((q=(yTOx100+Err100))/100) ;			//DY is Number of Y bits to set in this iteration
			Err100 = q % 100 ;								//Follow up on accumulated Error
			for (; DY ; DY -= pix, y += pix)
				pix = setYPixels( x, y, DY, lw ) ;			//Draw DY bits
		}
	}
	else {													//Is DX larger than DY?
		Dx100 = (int)dx*100 ;								//Yes. Iterate on Y
		xTOy100 = Dx100 / dy ;								//Draw xToy pixels each iteration

		for (x=x0, y=y0, Err100=0 ; y<=y1 ; ++y) {
			DX = (BYTE)((q=(xTOy100+Err100))/100) ;			//DX is number of X bits to set in this iteration
			Err100 = q % 100 ;								//Follow up on accumulated Error
			while (DX--) {
				setXPixel( x, y, lw );						//Draw DX bits
				x += stepX ;
			}
		}
	}

	if (Err100>50)
		setXPixel(x,y,lw) ;										//If remaining Error, Fix with an additional pixel

}
