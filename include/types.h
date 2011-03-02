#ifndef HJ_NODE_TYPES_H_
#define HJ_NODE_TYPES_H_

typedef double Covariance[64];


#ifdef __cplusplus

template <typename T, size_t N>
inline
size_t ARRAY_SIZE(const T (&lhs)[N])
{
	return N;
}

#else

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#endif

#endif
