#ifndef __cy_gpio_h
#define __cy_gpio_h












typedef struct {

	int a;
}cy_gpio_type;









extern cy_gpio_type cy_gpio_;











void cy_gpio_init(void);
void cy_gpio_polling(void);








#endif //__cy_gpio_h
