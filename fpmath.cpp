#include "fpmath.hpp"

char * fp::frac(std::int32_t fpd, std::uint8_t accuracy) noexcept
{
	assert(accuracy < 10);

	const auto intpow = [](std::uint32_t a, std::uint32_t b) noexcept -> std::uint32_t
	{
		switch (b)
		{
		case 0:
			return 1;
		case 1:
			return a;
		default:
			for (auto a1 = a; b > 1; --b)
			{
				a *= a1;
			}
			return a;
		}
	};

	auto accpow = intpow(10, accuracy);

	fpd &= FPD_MASK;
	std::uint32_t frac = std::uint32_t((std::uint64_t(fpd) * std::uint64_t(accpow)) / std::uint64_t(FPD_FACTOR));
	
	char * out = new char[10];
	out[0];
	char * p = out;

	accpow /= 10;
	while (accpow > frac)
	{
		*p = '0';
		++p;
		accpow /= 10;
	}
	*p = '\0';

	itoa(frac, p, 10);

	return out;
}
