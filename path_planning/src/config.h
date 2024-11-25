#ifndef CONFIG_H
#define CONFIG_H

#define MAP_SIZE 20.0
#define CELL_SIZE 0.1
#define PATH_CELL_SIZE 0.1
#define RENDER_SIZE 0.1

#define INTERVAL 3
#define LEARNING_RATE1 0.20
#define LEARNING_RATE2 0.12
#define LEARNING_RATE3 0.05
#define LEARNING_RATE4 0.02
#define LEARNING_RATE5 0.01
#define MAX_ITER 40
// #define DROPOUT 3

#define DT .02
#define VSTART 0.
#define VEND 0.
#define VMAX 6.
#define AMAX 3.
#define GAMMA .5
#define BETA 1e3
#define TOPP_ITER 250

#define NMPC 8

#endif  // CONFIG_H