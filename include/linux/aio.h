#ifndef __LINUX__AIO_H
#define __LINUX__AIO_H

#include <linux/aio_abi.h>

struct kioctx;
struct kiocb;
struct mm_struct;

#define KIOCB_KEY		0

typedef int (kiocb_cancel_fn)(struct kiocb *);

/* prototypes */
#ifdef CONFIG_AIO
extern void exit_aio(struct mm_struct *mm);
extern long do_io_submit(aio_context_t ctx_id, long nr,
			 struct iocb __user *__user *iocbpp, bool compat);
void kiocb_set_cancel_fn(struct kiocb *req, kiocb_cancel_fn *cancel);
struct mm_struct *aio_get_mm(struct kiocb *req);
struct task_struct *aio_get_task(struct kiocb *req);
struct iov_iter;
ssize_t generic_async_read_iter(struct kiocb *iocb, struct iov_iter *iter);
ssize_t generic_async_write_iter(struct kiocb *iocb, struct iov_iter *iter);
#else
static inline void exit_aio(struct mm_struct *mm) { }
static inline long do_io_submit(aio_context_t ctx_id, long nr,
				struct iocb __user * __user *iocbpp,
				bool compat) { return 0; }
static inline void kiocb_set_cancel_fn(struct kiocb *req,
				       kiocb_cancel_fn *cancel) { }
static inline struct mm_struct *aio_get_mm(struct kiocb *req) { return NULL; }
static inline struct task_struct *aio_get_task(struct kiocb *req) { return current; }
#endif /* CONFIG_AIO */

/* for sysctl: */
extern unsigned long aio_nr;
extern unsigned long aio_max_nr;
extern unsigned long aio_auto_threads;

#endif /* __LINUX__AIO_H */
