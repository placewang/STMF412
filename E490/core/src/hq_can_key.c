/*
 * 20180224
 *
 * select() event polling mechanism
 *
 */
#include <stdio.h>
#include <string.h>


int Get_Keydata_(unsigned short *key_p)
{
	unsigned short k1,k2,k3,k4,kt;
	unsigned char type_id=0;
	unsigned short key=*key_p;
	
	kt=0;
	k1 = key & 0x0F;
	k2 = (key>>4) & 0x0F;
	k3 = (key>>8) & 0x0F;
	k4 = (key>>12) & 0x0F;
	if ((key==0)||(key==0xffff))
	{
		return -1;		 
	}

	type_id = (k1 & 0x01)|(k2 & 0x02) |(k3 & 0x04)|(k4 & 0x08);

	switch (type_id & 0x07)
	{		
		case 1:
			{
				kt=k1;
				k1=k2;
				k2=kt;
			}
			break;
		case 2:
			{
				kt=k1;
				k1=k3;
				k3=kt;

			}
			break;
		case 3:
			{
				kt=k1;
				k1=k4;
				k4=kt;
			}
			break;
		case 4:
			{
				kt=k1;
				k1=k2;
				k2=kt;
				kt=k3;
				k3=k4;
				k4=kt;
			}
			break;
		case 5:
			{
				kt=k1;
				k1=k3;
				k3=kt;
				kt=k2;
				k2=k4;
				k4=kt;
			}
			break;
		case 6:
			{
				kt=k1;
				k1=k4;
				k4=kt;
				kt=k2;
				k2=k3;
				k3=kt;
			}
			break;
		case 7:
			{
				kt=k1;
				k1=k4;
				k4=k3;
				k3=k2;
				k2=kt;
				//k4=kt;
			}
			break;
		default:
			break;
	}
	k4 =~k4; 
	*key_p = (unsigned short)((k1 & 0x0f) | ((k2 & 0x0f)<<4) |((k3 & 0x0f)<<8)|((k4 & 0x0f)<<12));
	return 0;
	
}

int Encryption_Data_withKey_Short(unsigned short *candata,unsigned short key )
{
	int key_ok;
	unsigned short dt=*candata;
	int i;
	key_ok = Get_Keydata_(&key);
	if (key_ok ==0)
	{
		//ok
		unsigned short dt_2 =0;
		for (i =0;i<4;i++)
		{
			dt_2|=( ((((dt >>(i<<2)) & 0x0f)+(key & 0x0f)+i) & 0x0f) <<(i<<2));
		}
		dt = dt_2;
		dt_2=dt>>(0x10 -((key>>4) & 0x0f) );
		dt = dt<<((key>>4) & 0x0f);
		dt |=dt_2;
		
		dt_2 =0;
		for (i =0;i<8;i++)
		{			
			dt_2 |= (((key >> (8+i) )& 0x01)?(~((dt>>i ) & 0x01) &0x01):((dt>>i ) & 0x01))<<i;	
		}
		for (i =0;i<8;i++)
		{			
			dt_2 |= (((key >> (8+i) )& 0x01)?(~((dt>>(i+8) ) & 0x01) &0x01):((dt>>(i+8) ) & 0x01))<<(i+8);	
		}

		dt = dt_2;
		*candata = dt;
		return 0;
		
	}
	else
	return -1;	
	

}



int Decrypt_Data_withKey_Short(unsigned short *candata,unsigned short key )
{
	int key_ok;
	unsigned short dt=*candata;
	int i;
	key_ok = Get_Keydata_(&key);
	if (key_ok ==0)
	{
		//ok
		unsigned short dt_2 =0;

//dt_2 =0;
		for (i =0;i<8;i++)
		{			
			dt_2 |= (((key >> (8+i) )& 0x01)?(~((dt>>i ) & 0x01) &0x01):((dt>>i ) & 0x01))<<i;	
		}
		for (i =0;i<8;i++)
		{			
			dt_2 |= (((key >> (8+i) )& 0x01)?(~((dt>>(i+8) ) & 0x01) &0x01):((dt>>(i+8) ) & 0x01))<<(i+8);	
		}

		dt = dt_2;
		
		dt_2=dt<<(0x10 -((key>>4) & 0x0f) );
		dt = dt>>((key>>4) & 0x0f);
		dt |=dt_2;

		dt_2 =0;
		for (i =0;i<4;i++)
		{
			dt_2|=( ((((dt >>(i<<2)) & 0x0f)+0x10-(key & 0x0f)-i) & 0x0f)<<(i<<2));
		}
		
		
		*candata = dt_2;
		return 0;
		
	}
	else
	return -1;	
	

}

/**************
*	加密CAN数据
*	data_buff 由三个short 组成，data   ---希望读写的数据,
*									index ---希望读写的序号(0-63),
*									d_key ---第一层加密的密钥,0表示不加密。
*	加密完成之后的数据，依次放入data_buff；
*	syskey 用于第二层加密的密钥--为0表示不加密
*	return值，0--加密成功。!0 --表示失败。
*/
int Encryption_can_data(unsigned short *data_buff,unsigned short syskey)
{
	unsigned short d[3];
	unsigned short d_o;
	int i;
	int ret=0;
	
	memcpy((unsigned char *)d,(unsigned char *)data_buff,6);

	d_o = d[0];
	if ((d[2]!=0)&&(d[2]!=0xffff))/*data_key*/
	{
		if (Encryption_Data_withKey_Short(&d[0],d[2])!=0)  /*加密失败。那就不加密*/
		{
			d[2]=0;
		}
		else
			if (Encryption_Data_withKey_Short(&d[1],d[2])!=0)
			{
				d[2]=0;
				d[0]= d_o;					
			}			
	}

	if ((syskey!=0)&&(syskey!=0xffff))
	{
		for (i=0;i<3;i++)
		{
			if (Encryption_Data_withKey_Short(&d[i],syskey)!=0)
			{
				ret =1;					/*加密密钥失败*/				
			}
		}
	}

	memcpy((unsigned char *)data_buff,(unsigned char *)d,6);
	return ret;

}

/**************
*	解密CAN数据
*	data_buff 由三个short 组成，data   ---希望读写的数据,
*									index ---希望读写的序号(0-63),
*									d_key ---第一层加密的密钥,0表示不加密。
*	解密完成之后的数据，依次放入data_buff；
*	syskey 用于第二层加密的密钥--为0表示不加密，该值由命令(0x01-0x24-syskey)给定。
*	return值，0--解密成功。!0 --表示失败。
*/
int Decrypt_can_data(unsigned short *data_buff,unsigned short syskey)
{
	unsigned short d[3];
	//unsigned short d_o;
	int i;
	int ret=0;
	
	memcpy((unsigned char *)d,(unsigned char *)data_buff,6);
	if ((syskey!=0)&&(syskey!=0xffff))
	{
		for (i=0;i<3;i++)
		{
			if (Decrypt_Data_withKey_Short(&d[i],syskey)!=0)
			{
				ret =1;					/*解密密钥失败*/
				goto return_end;
			}
		}
	}

	if ((d[2]!=0)&&(d[2]!=0xffff))  /*d[2] is data_key*/
	{
		if (Decrypt_Data_withKey_Short(&d[1],d[2])!=0)
		{
				ret=2;					/*解密序号失败*/
				goto return_end;
		}
		else
		{
			if (Decrypt_Data_withKey_Short(&d[0],d[2])!=0)
			{
				ret=3;					/*解密数据失败*/
				goto return_end;
			}
		}
	}

	memcpy((unsigned char *)data_buff,(unsigned char *)d,6);
	
return_end:	
	return ret;

}




