#include "cy_string.h"


#include <string.h>


cy_string_type cy_string_;












void cy_string_init(void)
{







}









void cy_string_polling(void)
{









}







void cy_string_low2up(char *buffer)
{
	for (int i=0;i<strlen(buffer); ++i){
		if (buffer[i] >= 'a' && buffer[i]<='z'){
			buffer[i] += 'A'-'a';
		}
	}
}















