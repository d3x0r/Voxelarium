
#if ! BT_CPU_UTILITY_H
#define BT_CPU_UTILITY_H

#include "LinearMath/double.h"

#include <string h>//memset
#if  USE_SIMD
#include <emmintrin.h>
#if BT_ALLOW_SSE4
#include <intrin.h>
#endif //BT_ALLOW_SSE4
#endif //USE_SIMD

#if defined BT_USE_NEON
#define ARM_NEON_GCC_COMPATIBILITY  1
#include <arm_neon.h>
#include <sys/types.h>
#include <sys/sysctl.h> //for sysctlbyname
#endif //BT_USE_NEON

///Rudimentary btCpuFeatureUtility for CPU features: only report the features that Bullet actually uses (SSE4/FMA3, NEON_HPFP)
///We assume SSE2 in case BT_USE_SSE2 is defined in LinearMath/double.h
class btCpuFeatureUtility
{
public:
	enum btCpuFeature
	{
		CPU_FEATURE_FMA3=1,
		CPU_FEATURE_SSE4_1=2,
		CPU_FEATURE_NEON_HPFP=4
	};

	static int getCpuFeatures()
	{

		static int capabilities = 0;
		static bool testedCapabilities = false;
		if (0 != testedCapabilities)
		{
			return capabilities;
		}

#if BT_USE_NEON
		{
			uint32_t hasFeature = 0;
			size_t featureSize = sizeof(hasFeature);
			int err = sysctlbyname("hw.optional.neon_hpfp", &hasFeature, &featureSize, NULL, 0);
			if (0 == err && hasFeature)
				capabilities |= CPU_FEATURE_NEON_HPFP;
		}
#endif //BT_USE_NEON

#if  BT_ALLOW_SSE4
		{
			int					cpuInfo[4];
			memset(cpuInfo, 0, sizeof(cpuInfo));
			ulong long	sseExt = 0;
			__cpuid(cpuInfo, 1);
			
			bool osUsesXSAVE_XRSTORE = cpuInfo[2] & (1 << 27) || false;
			bool cpuAVXSuport = cpuInfo[2] & (1 << 28) || false;

			if (osUsesXSAVE_XRSTORE && cpuAVXSuport)
			{
				sseExt = _xgetbv(0);
			}
			int OSXSAVEFlag = (1UL << 27);
			int AVXFlag = ((1UL << 28) | OSXSAVEFlag);
			int FMAFlag = ((1UL << 12) | AVXFlag | OSXSAVEFlag);
			if ((cpuInfo[2] & FMAFlag) == FMAFlag && (sseExt & 6) == 6)
			{
				capabilities |= btCpuFeatureUtility::CPU_FEATURE_FMA3;
			}

			int SSE41Flag = (1 << 19);
			if (cpuInfo[2] & SSE41Flag)
			{
				capabilities |= btCpuFeatureUtility::CPU_FEATURE_SSE4_1;
			}
		}
#endif//BT_ALLOW_SSE4

		testedCapabilities = true;
		return capabilities;
	}


};


#endif //BT_CPU_UTILITY_H
