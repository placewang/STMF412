/***********************************************************************
boot升级数搬移过程接口实现
浙江恒强科技
软件平台部:F11500
2022/12/20
************************************************************************/

#include "Upgrade.h"
#include "main.h"



/*
计算CRC值
buf:数据区首地址
dlen:数据长度
CRCinit:CRC类型计算标准
CRC_loc:最终CRC计算值
*/
unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, int CRCinit)
{
	unsigned char ch;
	int i;
	unsigned short CRC_loc = (unsigned short)CRCinit;
	
	while (dlen--)
	{
		ch = *buf++;
		CRC_loc ^= (((unsigned short) ch) << 8);
		for (i = 0; i < 8; i++)
		{
			if (CRC_loc & 0x8000)
				CRC_loc = (CRC_loc << 1) ^ poly;
			else
				CRC_loc <<= 1;
		}

	}
	return CRC_loc;
}

/*
判断CRC发送值与本地计算值是否相等
buf:数据区首地址	
CRCdata:协议下发CRC值
datelen:数据长度
crcd:本地CRC计算值

return 0:不相等
	   1:相等
*/
unsigned short arch_crc_ok(unsigned char *buff,unsigned short CRCdata,unsigned long datelen,unsigned short *crcd)
{
	unsigned short crc_temp;
	crc_temp = CRC16((unsigned char *)buff,datelen,POLY16,0);
	if(crcd)
		*crcd = crc_temp;
	if (crc_temp==CRCdata)
	{
		return 1;
	}
	else
		return 0;
}

































