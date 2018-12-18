#include "cy_controller.h"

#include "../controller/cy_b.h"


cy_controller_type cy_controller_;












void cy_controller_init(void)
{



	cy_controller_.absolute_coordinate = true;//¾ø¶Ô×ø±êÏµ



}









void cy_controller_polling(void)
{









}


void cy_controller_move(float x_, float y_, float z_)
{
	
	if (cy_controller_.absolute_coordinate == true){
		cy_b_move_abs(x_,y_,z_);
	}else{
		
	}
	
}




