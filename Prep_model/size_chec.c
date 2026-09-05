#include "rf_model.h"
#include "stdio.h"
volatile int keep_alive;
int main(void) {
    float dummy[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    keep_alive = rf_model_predict(dummy, 15);  
    printf("keep_alive: %d\n", keep_alive);
}