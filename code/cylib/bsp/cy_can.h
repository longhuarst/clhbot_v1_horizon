#ifndef __cy_can_h
#define __cy_can_h



#include <stdint.h>
#include <stdbool.h>



#define CY_CAN_MAX (6)






typedef struct {

	float x;
	float y;
	bool updated;
}cy_can_type;




void cy_can_resend(void);





extern cy_can_type cy_can_[CY_CAN_MAX];











void cy_can_init(void);
void cy_can_polling(void);








#endif //__cy_can_h
