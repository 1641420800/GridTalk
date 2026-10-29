#ifndef GRIDTALK_PIPE_STDIO_H
#define GRIDTALK_PIPE_STDIO_H
/*
 * GridTalk 管道 stdio 适配: gtpipe_fopen 把管道包装成 FILE*, 之后
 * fprintf/fscanf/fputs 等标准IO直接用于管道。
 *
 * 后端自动探测(可用 GTPIPE_STDIO_BACKEND 覆盖):
 *   1 = fopencookie (newlib / glibc)   -- 完整FILE*语义
 *   2 = fdevopen   (avr-libc)          -- FILE*由fdevopen分配
 *   0 = 无后端 (如MSVC/Keil microlib)  -- gtpipe_fopen返回NULL;
 *       请使用 gtpipe_printf / gtpipe_read (核心组件, 任何平台可用)
 *
 * 语义注意:
 *   - 读方向(FILE读/fscanf)在无数据时自旋等待(GTPIPE_STDIO_TIMEOUT_MS超时后
 *     返回EOF, clearerr后可继续); 中断驱动的固件上自旋期间ISR推进协议;
 *   - 写方向为行缓冲("w"模式), 二进制/立即上线请fflush;
 *   - 每个包含本头的编译单元最多 gtpipe_fopen GTPIPE_STDIO_MAX 次(cookie池)。
 */
#include "gridtalk_pipe.h"

#ifndef GTPIPE_STDIO_BACKEND
  #if defined(__NEWLIB__) || defined(_NEWLIB_VERSION) || defined(__GLIBC__) || defined(_GNU_SOURCE)
    #define GTPIPE_STDIO_BACKEND 1
  #elif defined(__AVR_LIBC_VERSION__) || defined(__AVR_LIBC__)
    #define GTPIPE_STDIO_BACKEND 2
  #else
    #define GTPIPE_STDIO_BACKEND 0
  #endif
#endif

#ifndef GTPIPE_STDIO_TIMEOUT_MS
#define GTPIPE_STDIO_TIMEOUT_MS 60000u   /* FILE读自旋超时(ms), 超时按EOF处理 */
#endif
#ifndef GTPIPE_STDIO_MAX
#define GTPIPE_STDIO_MAX 4               /* cookie池容量(每编译单元) */
#endif

#if GTPIPE_STDIO_BACKEND == 1
/* ================= fopencookie (newlib / glibc) ================= */
#include <stdio.h>
#include <stdint.h>

typedef struct { gtpipe_t *p; } gtpipe_cookie_t;

static ssize_t gtpipe_ck_read(void *ck, char *buf, size_t n)
{
  gtpipe_cookie_t *c = (gtpipe_cookie_t *)ck;
  if(n > 0xFFFFu) n = 0xFFFFu;
  uint32_t t0 = c->p->h->gt->nowUs ? c->p->h->gt->nowUs() : c->p->h->gt->_nowUs;
  for(;;)
  {
    int got = gtpipe_read(c->p, buf, (uint16_t)n);
    if(got > 0) return (ssize_t)got;
    uint32_t now = c->p->h->gt->nowUs ? c->p->h->gt->nowUs() : c->p->h->gt->_nowUs;
    if((uint32_t)(now - t0) > GTPIPE_STDIO_TIMEOUT_MS * 1000u)
      return 0;   /* 超时=EOF; clearerr后可继续读 */
  }
}

static ssize_t gtpipe_ck_write(void *ck, const char *buf, size_t n)
{
  gtpipe_cookie_t *c = (gtpipe_cookie_t *)ck;
  uint32_t done = 0;
  uint32_t t0 = c->p->h->gt->nowUs ? c->p->h->gt->nowUs() : c->p->h->gt->_nowUs;
  while(done < n)
  {
    uint16_t chunk = (n - done > 0xFFFFu) ? 0xFFFFu : (uint16_t)(n - done);
    int put = gtpipe_write(c->p, buf + done, chunk);
    if(put > 0) done += (uint32_t)put;
    uint32_t now = c->p->h->gt->nowUs ? c->p->h->gt->nowUs() : c->p->h->gt->_nowUs;
    if(put <= 0 && (uint32_t)(now - t0) > GTPIPE_STDIO_TIMEOUT_MS * 1000u)
      return (ssize_t)done;   /* 超时: 已写字数(短写) */
  }
  return (ssize_t)n;
}

static int gtpipe_ck_close(void *ck)
{
  (void)ck;
  return 0;   /* cookie属静态池, 不真正释放 */
}

/* 打开管道为FILE*: mode "r"=读, "w"=写(行缓冲)。失败(cookie池满)返回NULL */
static FILE *gtpipe_fopen(gtpipe_t *p, const char *mode)
{
  static gtpipe_cookie_t s_ck[GTPIPE_STDIO_MAX];
  static uint8_t s_ckN;
  if(s_ckN >= GTPIPE_STDIO_MAX) return NULL;
  gtpipe_cookie_t *c = &s_ck[s_ckN++];
  c->p = p;
  cookie_io_functions_t fns;
  fns.read = gtpipe_ck_read;
  fns.write = gtpipe_ck_write;
  fns.seek = NULL;
  fns.close = gtpipe_ck_close;
  const char *m = (*mode == 'r') ? "r" : "w";
  FILE *fp = fopencookie((void *)c, m, fns);
  if(fp && *mode == 'w') (void)setvbuf(fp, NULL, _IOLBF, 256);   /* 行缓冲: 按行上线 */
  return fp;
}

#elif GTPIPE_STDIO_BACKEND == 2
/* ================= fdevopen (avr-libc) ================= */
#include <stdio.h>

static int gtpipe_avr_put(char ch, FILE *f)
{
  gtpipe_t *p = (gtpipe_t *)fdev_get_udata(f);
  while(gtpipe_write(p, &ch, 1) == 0) { }   /* 自旋至接受(中断驱动由ISR推进) */
  return (uint8_t)ch;
}

static int gtpipe_avr_get(FILE *f)
{
  gtpipe_t *p = (gtpipe_t *)fdev_get_udata(f);
  uint8_t ch;
  while(gtpipe_read(p, &ch, 1) == 0) { }
  return ch;
}

static FILE *gtpipe_fopen(gtpipe_t *p, const char *mode)
{
  FILE *f = fdevopen((*mode == 'w') ? gtpipe_avr_put : NULL,
                     (*mode == 'r') ? gtpipe_avr_get : NULL);
  if(f) fdev_set_udata(f, p);
  return f;
}

#else
/* ================= 无后端: gtpipe_printf / gtpipe_read 可用 ================= */
#include <stdio.h>
#define gtpipe_fopen(p, mode)  ((FILE *)0)   /* 编译通过占位, 运行时不可用 */
#endif

#endif /* GRIDTALK_PIPE_STDIO_H */
