#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#define PACKED __attribute__((packed))

// prefix attribute.
// marks the symbol "used" for the linker, so it won't be optimized away.
#define USED           __attribute__ ((used))

// prefix attribute.
// marks the symbol "used" for the linker, so there won't be warning about it
#define UNUSED           __attribute__ ((unused))

#define FORCEINLINE      __attribute__ ((always_inline))

#define VISIBILITY(x) __attribute__((visibility(x)))

#define ALIGNED(x) __attribute__((aligned(x)))

// postfix attribute.
// marks a function as an interrupt handler.
#define INTERRUPT      __attribute__((__interrupt__))

// prefix attribute.
// places a symbol into a certain section.
#define SECTION(_Name) __attribute__ ((section(_Name)))

// prefix attribute.
// marks the function to "not return"
#define NORETURN       __attribute__((noreturn))

#endif // ATTRIBUTES_H
