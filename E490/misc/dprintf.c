
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

volatile int printf_disable = 0;
void USART_Out(char data);

static void myprintchar(char **str, int c)
{
	if (str) {
		**str = c;
		++(*str);
	}
	else USART_Out(c);
}

#define FLOAT_SUPPORT

#define PAD_RIGHT 1
#define PAD_ZERO 2

#define FALSE 0
#define TRUE  1

static int myprints(char **out, const char *string, int width, int pad, int printlimit, char IsNumber)
{
	register int pc = 0, padchar = ' ';
	int i,len;

	if (width > 0) {
		register int len1 = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len1;
		if (len1 >= width) width = 0;
		else width -= len1;
		if (pad & PAD_ZERO) padchar = '0';
	}

	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			myprintchar (out, padchar);
			++pc;
		}
	}

	if( FALSE == IsNumber )
	{
		// The string to print is not the result of a number conversion to ascii.
		/* For a string, printlimit is the max number of characters to display. */
		for ( ; printlimit && *string ; ++string, --printlimit) {
			myprintchar (out, *string);
			++pc;
		}
	}

	if((TRUE == IsNumber) && (printlimit  > 0))
	{
		// The string to print represents an integer number.
		/* In this case, printlimit is the min number of digits to print. */
		/* If the length of the number to print is less than the min nb of i
		   digits to display, we add 0 before printing the number. */
		len = strlen(string);
		if( len < printlimit)
		{
			i = printlimit - len;
			for(; i; i--) {
				myprintchar (out, '0');
				++pc;
			}
		}
	}

	/* Else: The string to print is not the result of a number conversion to ascii.
	 * For a string, printlimit is the max number of characters to display.
	 */

	for ( ; printlimit && *string ; ++string, --printlimit) {
		myprintchar (out, *string);
		++pc;
	}

	for ( ; width > 0; --width) {
		myprintchar (out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 64//12
char print_buf[PRINT_BUF_LEN];

static int myprinti(char **out, int i, int b, int sg, int width, int pad, int letbase, int printlimit)
{
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return myprints (out, print_buf, width, pad, printlimit, TRUE);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			myprintchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + myprints (out, s, width, pad, printlimit, TRUE);
}

#ifdef FLOAT_SUPPORT
static int myprintFloat(char **out, float i)
{
	int ret;
	ret = myprinti(out, (int)i, 10, 1, 4, 0, 0, 0);
	ret += myprinti(out, (int)((i-(int)i)*1000),10,0, 4,0,0,0 );
	return ret;
}
#endif

int myprint(char **out, const char *format, va_list args ) //进行数据格式转化
{
	register int width, pad, printlimit;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = printlimit = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if (*format == '.') {
				++format;
				for ( ; *format >= '0' && *format <= '9'; ++format) {
					printlimit *= 10;
					printlimit += *format - '0';
				}
			}
			if( 0 == printlimit )
				printlimit--;
			if (*format == 'l') {
				++format;
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += myprints (out, s?s:"(null)", width, pad, printlimit, FALSE);
				continue;
			}
			if( *format == 'd' ) {
				pc += myprinti (out, va_arg( args, int ), 10, 1, width, pad, 'a', printlimit);
				continue;
			}
#ifdef FLOAT_SUPPORT
			if( *format == 'f'){
				pc += myprintFloat(out, va_arg( args, int ));
				continue;
			}
#endif
			if( *format == 'x' ) {
				pc += myprinti (out, va_arg( args, int ), 16, 0, width, pad, 'a', printlimit);
				continue;
			}
			if( *format == 'X' ) {
				pc += myprinti (out, va_arg( args, int ), 16, 0, width, pad, 'A', printlimit);
				continue;
			}
			if( *format == 'u' ) {
				pc += myprinti (out, va_arg( args, int ), 10, 0, width, pad, 'a', printlimit);
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += myprints (out, scr, width, pad, printlimit, FALSE);
				continue;
			}
		}
		else {
out:
			myprintchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

#define printf	myprintf
int printf(const char *format, ...)
{
	va_list args;

	if(printf_disable) return 0;
	va_start( args, format );
	return myprint( 0, format, args );
}


#if 1
int sprintf_ex(char *out, const char *format, ...)
{
	va_list args;

	//if(printf_disable) return 0;
	va_start( args, format );
	return myprint( &out, format, args );
}

#endif
//

int myatoi(char *str)  
{
	unsigned long val;
	char c, radix, s = 0;

	while ((c = *str) == ' ') str++;
	if (c == '-') {
		s = 1;
		c = *(++str);
	}
	if (c == '0') {
		c = *(++str);
		if (c <= ' ') {
			return 0;
		}
		if (c == 'x') {
			radix = 16;
			c = *(++str);
		} else {
			if (c == 'b') {
				radix = 2;
				c = *(++str);
			} else {
				if ((c >= '0')&&(c <= '9'))
					radix = 8;
				else
					return 0;
			}
		}
	} else {
		if ((c < '1')||(c > '9'))
			return 0;
		radix = 10;
	}
	val = 0;
	while (c > ' ') {
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			if (c <= 9) return 0;
		}
		if (c >= radix) return 0;
		val = val * radix + c;
		c = *(++str);
	}
	if (s) val = -val;
	return val;
} 

void printf_enable()
{
	printf_disable = 0;
}
