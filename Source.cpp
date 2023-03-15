#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#pragma pack(1)

typedef char CHAR;
typedef short SHORT;
typedef long LONG;

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;

typedef int HFILE;
typedef CHAR *LPSTR, *PSTR;

#define FALSE 0
#define TRUE 1

//Format information for device-independent bitmaps
typedef struct tagBITMAPINFOHEADER {
	DWORD      biSize;
	LONG       biWidth;
	LONG       biHeight;
	WORD       biPlanes;
	WORD       biBitCount;
	DWORD      biCompression;
	DWORD      biSizeImage;
	LONG       biXPelsPerMeter;
	LONG       biYPelsPerMeter;
	DWORD      biClrUsed;
	DWORD      biClrImportant;
} BITMAPINFOHEADER, *LPBITMAPINFOHEADER, *PBITMAPINFOHEADER;

typedef struct tagBITMAPFILEHEADER {
	WORD    bfType;
	DWORD   bfSize;
	WORD    bfReserved1;
	WORD    bfReserved2;
	DWORD   bfOffBits;
}  BITMAPFILEHEADER, *LPBITMAPFILEHEADER, *PBITMAPFILEHEADER;

static int Zig_Zag[8][8] = { { 0, 1, 5, 6, 14, 15, 27, 28 },
{ 2, 4, 7, 13, 16, 26, 29, 42 },
{ 3, 8, 12, 17, 25, 30, 41, 43 },
{ 9, 11, 18, 24, 37, 40, 44, 53 },
{ 10, 19, 23, 32, 39, 45, 52, 54 },
{ 20, 22, 33, 38, 46, 51, 55, 60 },
{ 21, 34, 37, 47, 50, 56, 59, 61 },
{ 35, 36, 48, 49, 57, 58, 62, 63 }
};
/* constants for the biCompression field */
#define BI_RGB        0L
#define BI_RLE8       1L
#define BI_RLE4       2L
#define BI_BITFIELDS  3L
//The RGBQUAD structure describes a color consisting of relative intensities of red, green, and blue.
typedef struct tagRGBQUAD {
	BYTE    rgbBlue;
	BYTE    rgbGreen;
	BYTE    rgbRed;
	BYTE    rgbReserved;
} RGBQUAD;
typedef RGBQUAD * LPRGBQUAD;
//Below should be include in Windef.h but in this project we can't use other library, so manually define them.
#define MAKEWORD(a, b)      ((WORD)(((BYTE)(a)) | ((WORD)((BYTE)(b))) << 8))//combine low byte and high byte to a word
#define MAKELONG(a, b)      ((LONG)(((WORD)(a)) | ((DWORD)((WORD)(b))) << 16))
#define LOWORD(l)           ((WORD)(l))
#define HIWORD(l)           ((WORD)(((DWORD)(l) >> 16) & 0xFFFF))
#define LOBYTE(w)           ((BYTE)(w))
#define HIBYTE(w)           ((BYTE)(((WORD)(w) >> 8) & 0xFF))

//macro definition
#define WIDTHBYTES(i)    ((i+31)/32*4)//??????????
#define PI 3.1415926535
//define return value of function
#define FUNCTION_OK 0
#define FUNCTION_MEMORY_ERROR 1
#define FUNCTION_FILE_ERROR 2
#define FUNCTION_FORMAT_ERROR 3

//////////////////////////////////////////////////
//Jpeg functions
BOOL Jpeg2Bmp(char *JpegFileName);
void error(int message);
int  InitializeTag();
void InitializeTable();
int  Decode();
int  DecodeMCUBlock();
int  HuffmanBlock(BYTE dchufindex, BYTE achufindex);
int  DecodeElement();
//MCU is 8*8 block.
void QtMCUComponent(short flag);
void QtBlock(short  *s, int * d, short flag);
void GetYUV(short flag);
void StoreBuffer();
BYTE ReadByte();
void Initialize_IDCT();
void IDCT(int * block);
void IDCT_rows(int * block);
void IDCT_columns(int * block);
//////////////////////////////////////////////////
//global variable declaration
BITMAPFILEHEADER   bf;
BITMAPINFOHEADER   bi;
//HPALETTE           hPalette=NULL;
//HBITMAP            hBitmap=NULL;
char *            hImgData = NULL;
DWORD              NumColors;
DWORD              LineBytes;
DWORD              ImgWidth = 0, ImgHeight = 0;
char*             lpPtr;
/* Cosine Transform Coefficients */
#define W1 2841 /* 2048*sqrt(2)*cos(1*pi/16) */
#define W2 2676 /* 2048*sqrt(2)*cos(2*pi/16) */
#define W3 2408 /* 2048*sqrt(2)*cos(3*pi/16) */
#define W5 1609 /* 2048*sqrt(2)*cos(5*pi/16) */
#define W6 1108 /* 2048*sqrt(2)*cos(6*pi/16) */
#define W7 565  /* 2048*sqrt(2)*cos(7*pi/16) */
//////////////////////////////////////////////////
//variables used in jpeg function
short   H_Y2U, V_Y2U, H_Y2V, V_Y2V;
short   Y_MCU, U_MCU, V_MCU;
short   SampleRate_Y_H, SampleRate_Y_V;
short   SampleRate_U_H, SampleRate_U_V;
short   SampleRate_V_H, SampleRate_V_V;
unsigned char   *JpegBuf;
unsigned char   *lp;
short   QTtable[3][64];
short   compute_num;
BYTE   compute_index[3];
BYTE      YDcIdx;
BYTE	  YAcIdx;
BYTE      UVDcIdx;
BYTE      UVAcIdx;
BYTE   HuffmanTableIdx;
short      *Ytable, *Utable, *Vtable;
BYTE   And[9] = { 0, 1, 3, 7, 0xf, 0x1f, 0x3f, 0x7f, 0xff };
short      code_position_table[4][16], code_length_table[4][16];
unsigned short value_table[4][256];
unsigned short huffman_max_value[4][16], huffman_min_value[4][16];
short   BitPosition, CurrentByte;
short   interval = 0;
short   Mbuffer[10 * 64];
int    QTMbuffer[10 * 64];
short   BlockBuffer[64];
short   ycoef, ucoef, vcoef;//y u v coeffcient
BOOL   IntervalFlag;
int    Y[4 * 64], U[4 * 64], V[4 * 64];
DWORD      sizei, sizej;
short   rrun, vvalue;
short    restart;
static  long iclip[1024];
static  long *iclp;


BOOL Jpeg2Bmp(char *JpegFileName, char *BmpFileName)
{
	printf("Start converting Jpeg to Bmp.\n");
	FILE*      fjpeg;
	DWORD          ImgSize;
	DWORD              BufferSize, JpegBufferSize;
	FILE*              fbmp;
	FILE*              IMGdata;
	void *      hJpegBuf;
	int       errormessage;
	DWORD i;
	LPBITMAPINFOHEADER ImgData;

	char * hImgData256;
	fopen_s(&fjpeg, JpegFileName, "rb");
	printf("Jpeg file opened.\n");
	//get jpg file length
	fseek(fjpeg, 0L, SEEK_END);
	JpegBufferSize = ftell(fjpeg);
	//rewind to the beginning of the file
	fseek(fjpeg, 0L, SEEK_SET);

	if ((hJpegBuf = malloc(JpegBufferSize)) == NULL)
	{
		fclose(fjpeg);
		error(FUNCTION_MEMORY_ERROR);

		return FALSE;
	}
	JpegBuf = (unsigned char  *)hJpegBuf;
	fread((unsigned char  *)hJpegBuf, sizeof(char), JpegBufferSize, fjpeg);
	fclose(fjpeg);

	InitializeTable();

	if ((errormessage = InitializeTag()) != FUNCTION_OK)
	{
		// GlobalUnlock(hJpegBuf);
		free(hJpegBuf);
		error(errormessage);
		return FALSE;
	}
	//create new bitmapfileheader and bitmapinfoheader
	memset((char *)&bf, 0, sizeof(BITMAPFILEHEADER));
	memset((char *)&bi, 0, sizeof(BITMAPINFOHEADER));
	//bi =bitmap info, bf = bitmap file
	bi.biSize = (DWORD)sizeof(BITMAPINFOHEADER);
	bi.biWidth = (LONG)(ImgWidth);
	bi.biHeight = (LONG)(ImgHeight);
	bi.biPlanes = 1;
	bi.biBitCount = 24;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;
	bi.biCompression = BI_RGB;
	NumColors = 0;
	printf("biWidth is %ld\n", bi.biWidth);
	printf("biBitCount is %ld\n", bi.biBitCount);
	LineBytes = (DWORD)WIDTHBYTES(bi.biWidth*bi.biBitCount);
	printf("LineBytes is %ld\n", LineBytes);
	ImgSize = (DWORD)LineBytes*bi.biHeight;//???????
	printf("size is %ld\n", ImgSize);
	bf.bfType = 0x4d42;
	int a = sizeof(BITMAPFILEHEADER);
	int b = sizeof(BITMAPINFOHEADER);
	int c = NumColors * sizeof(RGBQUAD);

	bf.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + NumColors * sizeof(RGBQUAD) + ImgSize;
	bf.bfOffBits = 54;//(DWORD)(NumColors*sizeof(RGBQUAD)+sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER));
	BufferSize = bf.bfSize - sizeof(BITMAPFILEHEADER);
	// printf("size is %ld\n",BufferSize);
	if ((hImgData = (char*)malloc(BufferSize)) == NULL)
	{
		//GlobalUnlock(hJpegBuf);
		free(hJpegBuf);
		error(FUNCTION_MEMORY_ERROR);
		error(FUNCTION_MEMORY_ERROR);
		return FALSE;
	}
	// ImgData=(LPBITMAPINFOHEADER)GlobalLock(hImgData); 
	ImgData = (LPBITMAPINFOHEADER)hImgData;
	memcpy(ImgData, (char *)&bi, sizeof(BITMAPINFOHEADER));
	lpPtr = (char *)ImgData + sizeof(BITMAPINFOHEADER);

	if ((SampleRate_Y_H == 0) || (SampleRate_Y_V == 0))
	{
		// GlobalUnlock(hJpegBuf);
		free(hJpegBuf);
		//GlobalUnlock(hImgData);
		free(hImgData);
		hImgData = NULL;
		error(FUNCTION_FORMAT_ERROR);
		return FALSE;
	}

	errormessage = Decode();
	if (errormessage == FUNCTION_OK)
	{
		fopen_s(&fbmp, BmpFileName, "wb");
		fwrite((LPSTR)&bf, sizeof(BITMAPFILEHEADER), 1, fbmp);
		fwrite((LPSTR)ImgData, sizeof(char), BufferSize, fbmp);
		fopen_s(&IMGdata, "1.txt", "wb");

		DWORD xx = ImgWidth*ImgHeight;
		if ((hImgData256 = (char *)malloc(xx)) == NULL)
		{
			//GlobalUnlock(hJpegBuf);
			free(hImgData256);
			error(FUNCTION_MEMORY_ERROR);
			error(FUNCTION_MEMORY_ERROR);
			error(FUNCTION_MEMORY_ERROR);
			return FALSE;
		}
		printf("Image data write successfully.\n");
		char * temp = hImgData256;
		for (i = 0; i < xx; i++)
		{
			i;
			char t3 = *lpPtr;
			t3 &= 0xE0;//M_APP0
			char t1 = *(lpPtr + 1);
			t1 = t1 >> 3;
			t1 &= 0x1c;
			char t2 = *(lpPtr + 2);
			t2 = t2 >> 6;
			t2 &= 0x03;
			char t4 = t3 + t1 + t2;
			*temp++ = t4;
			lpPtr = lpPtr + 3;
		}
		int count = fwrite(hImgData256, sizeof(char), xx, IMGdata);
		fclose(IMGdata);

		fclose(fbmp);
		free(hJpegBuf);
		printf("Output bmp file.\n");
		return TRUE;
	}
	else
	{
		free(hJpegBuf);
		free(hImgData);
		hImgData = NULL;
		error(errormessage);
		return FALSE;
	}
}

void error(int message)
{
	switch (message)
	{
	case FUNCTION_MEMORY_ERROR:
		printf("Error alloc memory\n!");
		break;
	case FUNCTION_FILE_ERROR:
		printf("File not found!\n");
		break;
	case FUNCTION_FORMAT_ERROR:
		printf("File format error!\n");
		break;
	}
}

int InitializeTag()
{
	BOOL finish = FALSE;
	BYTE id;
	short  llength;
	short  i, j, k;
	short  huftab1, huftab2;
	short  HuffmanTableIdx;
	BYTE hf_table_index;
	BYTE QTtable_index;
	BYTE comnum;

	unsigned char  *lptemp;
	short  ccount;

	lp = JpegBuf + 2;

	while (!finish) {
		id = *(lp + 1);
		lp += 2;
		switch (id) {
		case 0xe0://M_APP0=0xe0
			llength = MAKEWORD(*(lp + 1), *lp);
			lp += llength;
			break;
		case 0xdb://M_DQT=0xdb
			llength = MAKEWORD(*(lp + 1), *lp);
			QTtable_index = (*(lp + 2)) & 0x0f;
			lptemp = lp + 3;
			if (llength<80) {
				for (i = 0; i<64; i++)
					QTtable[QTtable_index][i] = (short)*(lptemp++);
			}
			else {
				for (i = 0; i<64; i++)
					QTtable[QTtable_index][i] = (short)*(lptemp++);
				QTtable_index = (*(lptemp++)) & 0x0f;
				for (i = 0; i<64; i++)
					QTtable[QTtable_index][i] = (short)*(lptemp++);
			}
			lp += llength;
			break;
		case 0xc0://M_SOF0=0*c0
			llength = MAKEWORD(*(lp + 1), *lp);
			ImgHeight = MAKEWORD(*(lp + 4), *(lp + 3));
			ImgWidth = MAKEWORD(*(lp + 6), *(lp + 5));
			compute_num = *(lp + 7);
			if ((compute_num != 1) && (compute_num != 3))
				return FUNCTION_FORMAT_ERROR;
			if (compute_num == 3) {
				compute_index[0] = *(lp + 8);
				SampleRate_Y_H = (*(lp + 9)) >> 4;
				SampleRate_Y_V = (*(lp + 9)) & 0x0f;
				Ytable = (short *)QTtable[*(lp + 10)];

				compute_index[1] = *(lp + 11);
				SampleRate_U_H = (*(lp + 12)) >> 4;
				SampleRate_U_V = (*(lp + 12)) & 0x0f;
				Utable = (short *)QTtable[*(lp + 13)];

				compute_index[2] = *(lp + 14);
				SampleRate_V_H = (*(lp + 15)) >> 4;
				SampleRate_V_V = (*(lp + 15)) & 0x0f;
				Vtable = (short *)QTtable[*(lp + 16)];
			}
			else {
				compute_index[0] = *(lp + 8);
				SampleRate_Y_H = (*(lp + 9)) >> 4;
				SampleRate_Y_V = (*(lp + 9)) & 0x0f;
				Ytable = (short *)QTtable[*(lp + 10)];

				compute_index[1] = *(lp + 8);
				SampleRate_U_H = 1;
				SampleRate_U_V = 1;
				Utable = (short *)QTtable[*(lp + 10)];

				compute_index[2] = *(lp + 8);
				SampleRate_V_H = 1;
				SampleRate_V_V = 1;
				Vtable = (short *)QTtable[*(lp + 10)];
			}
			lp += llength;
			break;
		case 0xc4://M_DHT=0xc4
			llength = MAKEWORD(*(lp + 1), *lp);
			if (llength<0xd0) {
				huftab1 = (short)(*(lp + 2)) >> 4;     //huftab1=0,1
				huftab2 = (short)(*(lp + 2)) & 0x0f;   //huftab2=0,1
				HuffmanTableIdx = huftab1 * 2 + huftab2;
				lptemp = lp + 3;
				for (i = 0; i<16; i++)
					code_length_table[HuffmanTableIdx][i] = (short)(*(lptemp++));
				j = 0;
				for (i = 0; i<16; i++)
					if (code_length_table[HuffmanTableIdx][i] != 0) {
						k = 0;
						while (k<code_length_table[HuffmanTableIdx][i]) {
							value_table[HuffmanTableIdx][k + j] = (short)(*(lptemp++));
							k++;
						}
						j += k;
					}
				i = 0;
				while (code_length_table[HuffmanTableIdx][i] == 0)
					i++;
				for (j = 0; j<i; j++) {
					huffman_min_value[HuffmanTableIdx][j] = 0;
					huffman_max_value[HuffmanTableIdx][j] = 0;
				}
				huffman_min_value[HuffmanTableIdx][i] = 0;
				huffman_max_value[HuffmanTableIdx][i] = code_length_table[HuffmanTableIdx][i] - 1;
				for (j = i + 1; j<16; j++) {
					huffman_min_value[HuffmanTableIdx][j] = (huffman_max_value[HuffmanTableIdx][j - 1] + 1) << 1;
					huffman_max_value[HuffmanTableIdx][j] = huffman_min_value[HuffmanTableIdx][j] + code_length_table[HuffmanTableIdx][j] - 1;
				}
				code_position_table[HuffmanTableIdx][0] = 0;
				for (j = 1; j<16; j++)
					code_position_table[HuffmanTableIdx][j] = code_length_table[HuffmanTableIdx][j - 1] + code_position_table[HuffmanTableIdx][j - 1];
				lp += llength;
			}  //if
			else {
				hf_table_index = *(lp + 2);
				lp += 2;
				while (hf_table_index != 0xff) {
					huftab1 = (short)hf_table_index >> 4;     //huftab1=0,1
					huftab2 = (short)hf_table_index & 0x0f;   //huftab2=0,1
					HuffmanTableIdx = huftab1 * 2 + huftab2;
					lptemp = lp + 1;
					ccount = 0;
					for (i = 0; i<16; i++) {
						code_length_table[HuffmanTableIdx][i] = (short)(*(lptemp++));
						ccount += code_length_table[HuffmanTableIdx][i];
					}
					ccount += 17;
					j = 0;
					for (i = 0; i<16; i++)
						if (code_length_table[HuffmanTableIdx][i] != 0) {
							k = 0;
							while (k<code_length_table[HuffmanTableIdx][i])
							{
								value_table[HuffmanTableIdx][k + j] = (short)(*(lptemp++));
								k++;
							}
							j += k;
						}
					i = 0;
					while (code_length_table[HuffmanTableIdx][i] == 0)
						i++;
					for (j = 0; j<i; j++) {
						huffman_min_value[HuffmanTableIdx][j] = 0;
						huffman_max_value[HuffmanTableIdx][j] = 0;
					}
					huffman_min_value[HuffmanTableIdx][i] = 0;
					huffman_max_value[HuffmanTableIdx][i] = code_length_table[HuffmanTableIdx][i] - 1;
					for (j = i + 1; j<16; j++) {
						huffman_min_value[HuffmanTableIdx][j] = (huffman_max_value[HuffmanTableIdx][j - 1] + 1) << 1;
						huffman_max_value[HuffmanTableIdx][j] = huffman_min_value[HuffmanTableIdx][j] + code_length_table[HuffmanTableIdx][j] - 1;
					}
					code_position_table[HuffmanTableIdx][0] = 0;
					for (j = 1; j<16; j++)
						code_position_table[HuffmanTableIdx][j] = code_length_table[HuffmanTableIdx][j - 1] + code_position_table[HuffmanTableIdx][j - 1];
					lp += ccount;
					hf_table_index = *lp;
				}  //while
			}  //else
			break;
		case 0xdd://M_DRI=0xdd
			llength = MAKEWORD(*(lp + 1), *lp);
			restart = MAKEWORD(*(lp + 3), *(lp + 2));
			lp += llength;
			break;
		case 0xda://M_SOS=0xda
			llength = MAKEWORD(*(lp + 1), *lp);
			comnum = *(lp + 2);
			if (comnum != compute_num)
				return FUNCTION_FORMAT_ERROR;
			lptemp = lp + 3;
			for (i = 0; i<compute_num; i++) {
				if (*lptemp == compute_index[0]) {
					YDcIdx = (*(lptemp + 1)) >> 4;   //Y
					YAcIdx = ((*(lptemp + 1)) & 0x0f) + 2;
				}
				else {
					UVDcIdx = (*(lptemp + 1)) >> 4;   //U,V
					UVAcIdx = ((*(lptemp + 1)) & 0x0f) + 2;
				}
				lptemp += 2;
			}
			lp += llength;
			finish = TRUE;
			break;
		case 0xd9://M_EOI=0xd9
			return FUNCTION_FORMAT_ERROR;
			break;
		default:
			if ((id & 0xf0) != 0xd0) {
				llength = MAKEWORD(*(lp + 1), *lp);
				lp += llength;
			}
			else lp += 2;
			break;
		}  //switch
	} //while
	return FUNCTION_OK;
}

void InitializeTable()
{
	short i, j;
	sizei = sizej = 0;
	ImgWidth = ImgHeight = 0;
	rrun = vvalue = 0;
	BitPosition = 0;
	CurrentByte = 0;
	IntervalFlag = FALSE;
	restart = 0;
	for (i = 0; i<3; i++)
		for (j = 0; j<64; j++)
			QTtable[i][j] = 0;
	compute_num = 0;
	HuffmanTableIdx = 0;
	for (i = 0; i<3; i++)
		compute_index[i] = 0;
	for (i = 0; i<4; i++)
		for (j = 0; j<16; j++) {
			code_length_table[i][j] = 0;
			code_position_table[i][j] = 0;
			huffman_max_value[i][j] = 0;
			huffman_min_value[i][j] = 0;
		}
	for (i = 0; i<4; i++)
		for (j = 0; j<256; j++)
			value_table[i][j] = 0;

	for (i = 0; i<10 * 64; i++) {
		Mbuffer[i] = 0;
		QTMbuffer[i] = 0;
	}
	for (i = 0; i<64; i++) {
		Y[i] = 0;
		U[i] = 0;
		V[i] = 0;
		BlockBuffer[i] = 0;
	}
	ycoef = ucoef = vcoef = 0;
}

int Decode()
{
	printf("Decoding the huffman table.\n");
	int errormessage;

	Y_MCU = SampleRate_Y_H*SampleRate_Y_V;
	U_MCU = SampleRate_U_H*SampleRate_U_V;
	V_MCU = SampleRate_V_H*SampleRate_V_V;
	H_Y2U = SampleRate_Y_H / SampleRate_U_H;
	V_Y2U = SampleRate_Y_V / SampleRate_U_V;
	H_Y2V = SampleRate_Y_H / SampleRate_V_H;
	V_Y2V = SampleRate_Y_V / SampleRate_V_V;
	Initialize_IDCT();
	printf("Start converting to quantized tables.(by applying reverse Zig_Zag)\n");
	printf("Start converting quantized tables to original DCT values.(by multiplying quantization table.\n");
	printf("Start applying Iverse Discrete cosine transform.\n");
	while ((errormessage = DecodeMCUBlock()) == FUNCTION_OK) {
		interval++;
		if ((restart) && (interval % restart == 0))
			IntervalFlag = TRUE;
		else
			IntervalFlag = FALSE;
		QtMCUComponent(0);//For Y
		QtMCUComponent(1);//For U
		QtMCUComponent(2);//For V
		GetYUV(0);
		GetYUV(1);
		GetYUV(2);
		StoreBuffer();
		sizej += SampleRate_Y_H * 8;
		if (sizej >= ImgWidth) {
			sizej = 0;
			sizei += SampleRate_Y_V * 8;
		}
		if ((sizej == 0) && (sizei >= ImgHeight))
			break;
	}
	return errormessage;
}

void  GetYUV(short flag)
{
	short H, VV;//VV for vertical, Because V is using at YUV
	short i, j, k, h;
	int  *buf;
	int  *pQtZzMCU;
	buf = Y;
	pQtZzMCU = QTMbuffer;
	switch (flag) {
	case 0:
		H = SampleRate_Y_H;
		VV = SampleRate_Y_V;
		buf = Y;
		pQtZzMCU = QTMbuffer;
		break;
	case 1:
		H = SampleRate_U_H;
		VV = SampleRate_U_V;
		buf = U;
		pQtZzMCU = QTMbuffer + Y_MCU * 64;
		break;
	case 2:
		H = SampleRate_V_H;
		VV = SampleRate_V_V;
		buf = V;
		pQtZzMCU = QTMbuffer + (Y_MCU + U_MCU) * 64;
		break;
	}
	for (i = 0; i<VV; i++)
		for (j = 0; j<H; j++)
			for (k = 0; k<8; k++)
				for (h = 0; h<8; h++)
					buf[(i * 8 + k)*SampleRate_Y_H * 8 + j * 8 + h] = *pQtZzMCU++;
}

void StoreBuffer()
{
	short i, j;
	unsigned char  *lpbmp;
	unsigned char R, G, B;
	int y, u, v, rr, gg, bb;

	for (i = 0; i<SampleRate_Y_V * 8; i++) {
		if ((sizei + i)<ImgHeight) {
			lpbmp = ((unsigned char *)lpPtr + (DWORD)(ImgHeight - sizei - i - 1)*LineBytes + sizej * 3);
			for (j = 0; j<SampleRate_Y_H * 8; j++) {
				if ((sizej + j)<ImgWidth) {
					y = Y[i * 8 * SampleRate_Y_H + j];
					u = U[(i / V_Y2U) * 8 * SampleRate_Y_H + j / H_Y2U];
					v = V[(i / V_Y2V) * 8 * SampleRate_Y_H + j / H_Y2V];
					rr = ((y << 8) + 18 * u + 367 * v) >> 8;
					gg = ((y << 8) - 159 * u - 220 * v) >> 8;
					bb = ((y << 8) + 411 * u - 29 * v) >> 8;
					R = (unsigned char)rr;
					G = (unsigned char)gg;
					B = (unsigned char)bb;
					if (rr & 0xffffff00) if (rr>255) R = 255; else if (rr<0) R = 0;
					if (gg & 0xffffff00) if (gg>255) G = 255; else if (gg<0) G = 0;
					if (bb & 0xffffff00) if (bb>255) B = 255; else if (bb<0) B = 0;
					*lpbmp++ = B;
					*lpbmp++ = G;
					*lpbmp++ = R;


				}
				else  break;
			}
		}
		else break;
	}
}

int DecodeMCUBlock()
{
	short *lpMbuffer;
	short i, j;
	int errormessage;

	if (IntervalFlag) {
		lp += 2;
		ycoef = ucoef = vcoef = 0;
		BitPosition = 0;
		CurrentByte = 0;
	}
	switch (compute_num) {
	case 3:
		lpMbuffer = Mbuffer;
		for (i = 0; i<SampleRate_Y_H*SampleRate_Y_V; i++)  //Y
		{
			errormessage = HuffmanBlock(YDcIdx, YAcIdx);
			if (errormessage != FUNCTION_OK)
				return errormessage;
			BlockBuffer[0] = BlockBuffer[0] + ycoef;
			ycoef = BlockBuffer[0];
			for (j = 0; j<64; j++)
				*lpMbuffer++ = BlockBuffer[j];
		}
		for (i = 0; i<SampleRate_U_H*SampleRate_U_V; i++)  //U
		{
			errormessage = HuffmanBlock(UVDcIdx, UVAcIdx);
			if (errormessage != FUNCTION_OK)
				return errormessage;
			BlockBuffer[0] = BlockBuffer[0] + ucoef;
			ucoef = BlockBuffer[0];
			for (j = 0; j<64; j++)
				*lpMbuffer++ = BlockBuffer[j];
		}
		for (i = 0; i<SampleRate_V_H*SampleRate_V_V; i++)  //V
		{
			errormessage = HuffmanBlock(UVDcIdx, UVAcIdx);
			if (errormessage != FUNCTION_OK)
				return errormessage;
			BlockBuffer[0] = BlockBuffer[0] + vcoef;
			vcoef = BlockBuffer[0];
			for (j = 0; j<64; j++)
				*lpMbuffer++ = BlockBuffer[j];
		}
		break;
	case 1:
		lpMbuffer = Mbuffer;
		errormessage = HuffmanBlock(YDcIdx, YAcIdx);
		if (errormessage != FUNCTION_OK)
			return errormessage;
		BlockBuffer[0] = BlockBuffer[0] + ycoef;
		ycoef = BlockBuffer[0];
		for (j = 0; j<64; j++)
			*lpMbuffer++ = BlockBuffer[j];
		for (i = 0; i<128; i++)
			*lpMbuffer++ = 0;
		break;
	default:
		return FUNCTION_FORMAT_ERROR;
	}
	return FUNCTION_OK;
}

int HuffmanBlock(BYTE dchufindex, BYTE achufindex)
{
	short count = 0;
	short i;
	int errormessage;

	//dc
	HuffmanTableIdx = dchufindex;
	errormessage = DecodeElement();
	if (errormessage != FUNCTION_OK)
		return errormessage;

	BlockBuffer[count++] = vvalue;
	//ac
	HuffmanTableIdx = achufindex;
	while (count<64) {
		errormessage = DecodeElement();
		if (errormessage != FUNCTION_OK)
			return errormessage;
		if ((rrun == 0) && (vvalue == 0)) {
			for (i = count; i<64; i++)
				BlockBuffer[i] = 0;
			count = 64;
		}
		else {
			for (i = 0; i<rrun; i++)
				BlockBuffer[count++] = 0;
			BlockBuffer[count++] = vvalue;
		}
	}
	return FUNCTION_OK;
}

int DecodeElement()
{
	int thiscode, tempcode;
	unsigned short temp, valueex;
	short codelen;
	BYTE hufexbyte, runsize, tempsize, sign;
	BYTE newbyte, lastbyte;

	if (BitPosition >= 1) {
		BitPosition--;
		thiscode = (BYTE)CurrentByte >> BitPosition;
		CurrentByte = CurrentByte&And[BitPosition];
	}
	else {
		lastbyte = ReadByte();
		BitPosition--;
		newbyte = CurrentByte&And[BitPosition];
		thiscode = lastbyte >> 7;
		CurrentByte = newbyte;
	}
	codelen = 1;
	while ((thiscode<huffman_min_value[HuffmanTableIdx][codelen - 1]) ||
		(code_length_table[HuffmanTableIdx][codelen - 1] == 0) ||
		(thiscode>huffman_max_value[HuffmanTableIdx][codelen - 1]))
	{
		if (BitPosition >= 1) {
			BitPosition--;
			tempcode = (BYTE)CurrentByte >> BitPosition;
			CurrentByte = CurrentByte&And[BitPosition];
		}
		else {
			lastbyte = ReadByte();
			BitPosition--;
			newbyte = CurrentByte&And[BitPosition];
			tempcode = (BYTE)lastbyte >> 7;
			CurrentByte = newbyte;
		}
		thiscode = (thiscode << 1) + tempcode;
		codelen++;
		if (codelen>16)
			return FUNCTION_FORMAT_ERROR;
	}  //while
	temp = thiscode - huffman_min_value[HuffmanTableIdx][codelen - 1] + code_position_table[HuffmanTableIdx][codelen - 1];
	hufexbyte = (BYTE)value_table[HuffmanTableIdx][temp];
	rrun = (short)(hufexbyte >> 4);
	runsize = hufexbyte & 0x0f;
	if (runsize == 0) {
		vvalue = 0;
		return FUNCTION_OK;
	}
	tempsize = runsize;
	if (BitPosition >= runsize) {
		BitPosition -= runsize;
		valueex = (BYTE)CurrentByte >> BitPosition;
		CurrentByte = CurrentByte&And[BitPosition];
	}
	else {
		valueex = CurrentByte;
		tempsize -= BitPosition;
		while (tempsize>8) {
			lastbyte = ReadByte();
			valueex = (valueex << 8) + (BYTE)lastbyte;
			tempsize -= 8;
		}  //while
		lastbyte = ReadByte();
		BitPosition -= tempsize;
		valueex = (valueex << tempsize) + (lastbyte >> BitPosition);
		CurrentByte = lastbyte&And[BitPosition];
	}  //else
	sign = valueex >> (runsize - 1);
	if (sign)
		vvalue = valueex;
	else {
		valueex = valueex ^ 0xffff;
		temp = 0xffff << runsize;
		vvalue = -(short)(valueex^temp);
	}
	return FUNCTION_OK;
}

void QtMCUComponent(short flag)
{
	short H, VV;
	short i, j;
	int *pQTMbuffer;
	short  *pMbuffer;
	pMbuffer = Mbuffer;
	pQTMbuffer = QTMbuffer;
	switch (flag) {
	case 0:
		H = SampleRate_Y_H;
		VV = SampleRate_Y_V;
		pMbuffer = Mbuffer;
		pQTMbuffer = QTMbuffer;
		break;
	case 1:
		H = SampleRate_U_H;
		VV = SampleRate_U_V;
		pMbuffer = Mbuffer + Y_MCU * 64;
		pQTMbuffer = QTMbuffer + Y_MCU * 64;
		break;
	case 2:
		H = SampleRate_V_H;
		VV = SampleRate_V_V;
		pMbuffer = Mbuffer + (Y_MCU + U_MCU) * 64;
		pQTMbuffer = QTMbuffer + (Y_MCU + U_MCU) * 64;
		break;
	}
	for (i = 0; i<VV; i++)
		for (j = 0; j<H; j++)
			QtBlock(pMbuffer + (i*H + j) * 64, pQTMbuffer + (i*H + j) * 64, flag);
}

void QtBlock(short  *s, int * d, short flag)
{
	short i, j;
	short tag;
	short *pQt;
	int buffer2[8][8];
	int *buffer1;
	short offset;
	pQt = Ytable;
	switch (flag) {
	case 0:
		pQt = Ytable;
		offset = 128;
		break;
	case 1:
		pQt = Utable;
		offset = 0;
		break;
	case 2:
		pQt = Vtable;
		offset = 0;
		break;
	}

	for (i = 0; i<8; i++)
		for (j = 0; j<8; j++) {
			tag = Zig_Zag[i][j];
			buffer2[i][j] = (int)s[tag] * (int)pQt[tag];
		}
	buffer1 = (int *)buffer2;
	IDCT(buffer1);
	for (i = 0; i<8; i++)
		for (j = 0; j<8; j++)
			d[i * 8 + j] = buffer2[i][j] + offset;
}


BYTE  ReadByte()
{
	BYTE  i;

	i = *(lp++);
	if (i == 0xff)
		lp++;
	BitPosition = 8;
	CurrentByte = i;
	return i;
}


//IDCT=inverse Discrete cosine transform
void IDCT(int * block)
{
	short i;

	for (i = 0; i<8; i++)
		IDCT_rows(block + 8 * i);

	for (i = 0; i<8; i++)
		IDCT_columns(block + i);
}


void Initialize_IDCT()
{
	short i;

	iclp = iclip + 512;
	for (i = -512; i<512; i++)
		iclp[i] = (i<-256) ? -256 : ((i>255) ? 255 : i);
}

void IDCT_rows(int * block)
{
	int x0, x1, x2, x3, x4, x5, x6, x7, x8;
	//intcut
	if (!((x1 = block[4] << 11) | (x2 = block[6]) | (x3 = block[2]) |
		(x4 = block[1]) | (x5 = block[7]) | (x6 = block[5]) | (x7 = block[3])))
	{
		block[0] = block[1] = block[2] = block[3] = block[4] = block[5] = block[6] = block[7] = block[0] << 3;
		return;
	}
	x0 = (block[0] << 11) + 128; // for proper rounding in the fourth stage 
								 //first stage
	x8 = W7*(x4 + x5);
	x4 = x8 + (W1 - W7)*x4;
	x5 = x8 - (W1 + W7)*x5;
	x8 = W3*(x6 + x7);
	x6 = x8 - (W3 - W5)*x6;
	x7 = x8 - (W3 + W5)*x7;
	//second stage
	x8 = x0 + x1;
	x0 -= x1;
	x1 = W6*(x3 + x2);
	x2 = x1 - (W2 + W6)*x2;
	x3 = x1 + (W2 - W6)*x3;
	x1 = x4 + x6;
	x4 -= x6;
	x6 = x5 + x7;
	x5 -= x7;
	//third stage
	x7 = x8 + x3;
	x8 -= x3;
	x3 = x0 + x2;
	x0 -= x2;
	x2 = (181 * (x4 + x5) + 128) >> 8;
	x4 = (181 * (x4 - x5) + 128) >> 8;
	//fourth stage
	block[0] = (x7 + x1) >> 8;
	block[1] = (x3 + x2) >> 8;
	block[2] = (x0 + x4) >> 8;
	block[3] = (x8 + x6) >> 8;
	block[4] = (x8 - x6) >> 8;
	block[5] = (x0 - x4) >> 8;
	block[6] = (x3 - x2) >> 8;
	block[7] = (x7 - x1) >> 8;
}

void IDCT_columns(int * block)
{
	int x0, x1, x2, x3, x4, x5, x6, x7, x8;
	//intcut
	if (!((x1 = (block[8 * 4] << 8)) | (x2 = block[8 * 6]) | (x3 = block[8 * 2]) |
		(x4 = block[8 * 1]) | (x5 = block[8 * 7]) | (x6 = block[8 * 5]) | (x7 = block[8 * 3])))
	{
		block[8 * 0] = block[8 * 1] = block[8 * 2] = block[8 * 3] = block[8 * 4] = block[8 * 5]
			= block[8 * 6] = block[8 * 7] = iclp[(block[8 * 0] + 32) >> 6];
		return;
	}
	x0 = (block[8 * 0] << 8) + 8192;
	//first stage
	x8 = W7*(x4 + x5) + 4;
	x4 = (x8 + (W1 - W7)*x4) >> 3;
	x5 = (x8 - (W1 + W7)*x5) >> 3;
	x8 = W3*(x6 + x7) + 4;
	x6 = (x8 - (W3 - W5)*x6) >> 3;
	x7 = (x8 - (W3 + W5)*x7) >> 3;
	//second stage
	x8 = x0 + x1;
	x0 -= x1;
	x1 = W6*(x3 + x2) + 4;
	x2 = (x1 - (W2 + W6)*x2) >> 3;
	x3 = (x1 + (W2 - W6)*x3) >> 3;
	x1 = x4 + x6;
	x4 -= x6;
	x6 = x5 + x7;
	x5 -= x7;
	//third stage
	x7 = x8 + x3;
	x8 -= x3;
	x3 = x0 + x2;
	x0 -= x2;
	x2 = (181 * (x4 + x5) + 128) >> 8;
	x4 = (181 * (x4 - x5) + 128) >> 8;
	//fourth stage
	block[8 * 0] = iclp[(x7 + x1) >> 14];
	block[8 * 1] = iclp[(x3 + x2) >> 14];
	block[8 * 2] = iclp[(x0 + x4) >> 14];
	block[8 * 3] = iclp[(x8 + x6) >> 14];
	block[8 * 4] = iclp[(x8 - x6) >> 14];
	block[8 * 5] = iclp[(x0 - x4) >> 14];
	block[8 * 6] = iclp[(x3 - x2) >> 14];
	block[8 * 7] = iclp[(x7 - x1) >> 14];
}

int main()
{
	char input[100];
	char output[100];
	printf("      # ######  #######  #####                     ######  #     # ######\n");
	printf("      # #     # #       #     #    #####  ####     #     # ##   ## #     # \n");
	printf("      # #     # #       #            #   #    #    #     # # # # # #     # \n");
	printf("      # ######  #####   #  ####      #   #    #    ######  #  #  # ######  \n");
	printf("#     # #       #       #     #      #   #    #    #     # #     # #       \n");
	printf("#     # #       #       #     #      #   #    #    #     # #     # #       \n");
	printf(" #####  #       #######  #####       #    ####     ######  #     # #       \n");
	printf("---------------------------------------------------------------------------\n");
	printf("Produce by Jiawei Dong, Rene Dominguez,Patrick Bishop.\n");
	printf("---------------------------------------------------------------------------\n\n\n");
	printf("Input your jpeg file location:\n");
	scanf("%s", input);
	printf("Input your output bmp file location:\n");
	scanf("%s", output);
	Jpeg2Bmp(input, output);
	system("pause");
	return 0;
}