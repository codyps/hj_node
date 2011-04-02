#ifndef HJ_NODE_TYPES_H_
#define HJ_NODE_TYPES_H_

#ifdef __cplusplus
//typedef double Covariance[36];
typedef boost::array<double, 36> Covariance;

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
