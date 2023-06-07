#ifndef __UTILITY_H__
#define __UTILITY_H__

char* _my_itoa(int num, char* str,int radix);

#define itoa(num,str,radix)  _my_itoa(num,str,radix)
#endif

