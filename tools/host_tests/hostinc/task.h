#ifndef TASK_HOST_STUB_H
#define TASK_HOST_STUB_H
/* Host stub: single-threaded test, so the critical sections are no-ops. */
#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)
#endif
