#ifndef __cy_gcoder_h
#define __cy_gcoder_h




#include <stdint.h>
#include <stdbool.h>








typedef struct {

	int a;
}cy_gcoder_type;









extern cy_gcoder_type cy_gcoder_;











void cy_gcoder_init(void);
void cy_gcoder_polling(void);








#endif //__cy_gcoder_h
