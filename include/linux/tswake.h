#ifndef _TSWAKE_H
#define _TSWAKE_H
/* input & i2c drivers need to register a few bits with tswake */
void tswake_notify_pwrkey(struct input_dev *dev);
void tswake_notify_resolution(int x, int y);
void tswake_notify_i2c(struct device *dev);

/* peek at tswake configuration */
int tswake_active(void);

/* inform tswake of a screen-off ts event */
void tswake_touch_event(int x, int y);

/* resume the i2c driver */
void tswake_i2c_enable(void);
#else
static inline void tswake_notify_pwrkey(struct input_dev *dev) { }
static inline void tswake_notify_resolution(int x, int y) { }
static inline void tswake_notify_i2c(struct device *dev) { }
static inline int tswake_active(void) { return 0; }
static inline int tswake_enabled(void) { return 0; }
static inline void tswake_touch_event(int x, int y) { }
void tswake_i2c_enable(void) { }
#endif
