#ifndef __cy_string_h
#define __cy_string_h








#include <stdint.h>
#include <stdbool.h>








typedef struct {

	int a;
}cy_string_type;









extern cy_string_type cy_string_;











void cy_string_init(void);
void cy_string_polling(void);


void cy_string_low2up(char *buffer);





#endif //__cy_string_h
