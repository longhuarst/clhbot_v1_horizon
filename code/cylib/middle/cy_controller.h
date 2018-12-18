#ifndef __cy_controller_h
#define __cy_controller_h





#include <stdint.h>
#include <stdbool.h>








typedef struct {

	bool absolute_coordinate;//¾ø¶Ô×ø±êÏµ
	float x;
	float y;
	float z;
}cy_controller_type;









extern cy_controller_type cy_controller_;











void cy_controller_init(void);
void cy_controller_polling(void);


void cy_controller_move(float x_, float y_, float z_);





#endif //__cy_controller_h
