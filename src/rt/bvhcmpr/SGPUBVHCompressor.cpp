
SGPUBVHCompressor::SGPUBVHCompressor(BVH& bvh, const BVH::BuildParams& params)
	: m_bvh(bvh),
	m_platform(bvh.getPlatform()),
	m_params(params),
	m_minOverlap(0.0f),
	m_sortDim(-1)
{
}

//------------------------------------------------------------------------

SGPUBVHCompressor::~SGPUBVHCompressor(void)
{
}

//------------------------------------------------------------------------

BVHNode* SGPUBVHCompressor::run(void)
{

}

#if 0

// Could make rule that precisely 32 children per node
// Could do extra splits if needed to pad them out


#include "Math/MiscMath.h"
#include "Math/Random.h"
#include "Util/Assert.h"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>
#include <algorithm>

struct FloatRep
{
	int signbits;
	int expobits;
	int mantbits;
	int excess;
	bool zeroone;
};

std::ostream& operator<<(std::ostream& os, const FloatRep& F)
{
	os << "[signbits:" << F.signbits << " expobits:" << F.expobits << " mantbits:" << F.mantbits << " excess:" << F.excess << " zeroone:" << F.zeroone << ']';
	return os;
}

// Reinterpret this unsigned int as a float
inline float uireinf(unsigned int x)
{
	float f = *(float*)(&x);
	return f;
}

// Return a random float with all bit patterns being equally probable
float randfloat()
{
	while (1) {
		unsigned int r = (rand() << 15) ^ (rand() << 30) ^ rand();
		float f = uireinf(r);
		if (dmcm::isFinite(f) && !dmcm::isNaN(f))
			return f;
	}
}

// Construct a float from its parts represented as ints
// zeroone uses the e=0,m=0 to represent 1.0, rather than 0.0.
float tofl(int sign, int expo, int mant, FloatRep& F)
{
	float signfl = (F.signbits > 0 && sign == 1) ? -1.0f : 1.0f;
	// float F.excess = powf(2.0f, (float)(F.expobits - 1)) - 1.0f;
	float expofl = expo - F.excess + ((expo > 0) ? 0 : 1);
	float mantfl = ((expo > 0) ? 1.0f : 0.0f) + (float)mant / powf(2.0f, (float)F.mantbits);
	float val = signfl * mantfl * powf(2.0f, expofl);

	if (F.zeroone && val == 0.0f)
		val = 1.0f;

	return val;
}

void getrep(float f, int& sign, int& expo, int& mant, FloatRep& F, bool roundup)
{
	if (F.signbits < 1) {
		f = f < 0 ? 0 : f;
	}

	sign = f < 0;
	int keepexpo = 0, keepmant = 0;
	float keepd = std::numeric_limits<float>::max();

	for (expo = 0; expo < (1 << F.expobits); expo++) {
		for (mant = 0; mant < (1 << F.mantbits); mant++) {
			float g = tofl(sign, expo, mant, F);
			float d = fabsf(g - f);

			if (d < keepd) {
				if ((roundup && g >= f) || (!roundup && g <= f)) {
					keepexpo = expo;
					keepmant = mant;
					keepd = d;
				}
			}
		}
	}

	expo = keepexpo;
	mant = keepmant;
}

void getrep0(float f, int& sign, int& expo, int& mant, FloatRep& F, bool roundup)
{
	if (F.signbits < 1) {
		f = f < 0 ? 0 : f;
	}

	sign = f < 0;
	float af = sign ? -f : f;
	int keepexpo, keepmant;

	for (expo = keepexpo = 0; expo < (1 << F.expobits); expo++) {
		for (mant = keepmant = 0; mant < (1 << F.mantbits); mant++) {
			float g = tofl(sign, expo, mant, F);

			if ((g < f && sign) || (g > f && !sign)) {
				if ((roundup && sign) || (!roundup && !sign)) {
					expo = keepexpo;
					mant = keepmant;
				}
				return;
			}
			keepexpo = expo;
			keepmant = mant;
		}
	}

	expo = keepexpo;
	mant = keepmant;
}

void showvals(FloatRep& F)
{
	for (int sign = 0; sign < (1 << F.signbits); sign++) {
		for (int expo = 0; expo < (1 << F.expobits); expo++) {
			for (int mant = 0; mant < (1 << F.mantbits); mant++) {
				float g = tofl(sign, expo, mant, F);
				std::cerr << "g:" << g << "\tsign:" << sign << " expo:" << expo << " mant:" << mant << " " << F << '\n';
			}
		}
	}
}

void test_getrep(FloatRep& F, bool roundup)
{
	int sign = 99, expo = 99, mant = 99;

	while (1) {
		float f = DRand();
		getrep(f, sign, expo, mant, F, roundup);
		float g = tofl(sign, expo, mant, F);
		std::cerr << f << "\tg:" << g << "\tsign:" << sign << " expo:" << expo << " mant:" << mant << " " << F << " roundup:" << roundup << '\n';
		ASSERT_D(g >= f && roundup || (g <= f && !roundup));
	}
}

class Span {
public:

	FloatRep parentRep, childRep, leafRep;

	Span(const FloatRep& parentRep_, const FloatRep& childRep_, const FloatRep& leafRep_) : parentRep(parentRep_), childRep(childRep_), leafRep(leafRep_) {}

	// Given the ROUNDED parent span and the two PRECISE child partitions, compute representation of child partitions and their global rounded coords
	void childSplits(float bll, float brr, float blr, float brl, float& lrfe, float& rlfe)
	{
		int sign = 99, expo = 99, mant = 99;

		ASSERT_D(brr >= bll);
		ASSERT_D(blr > bll && blr <= brr);
		ASSERT_D(brl >= bll && brl < brr);
		float wid = brr - bll;
		float lw = blr - bll;
		float rw = brr - brl;
		float lf = lw / wid;
		float rf = rw / wid;
		getrep(lf, sign, expo, mant, childRep, true);
		lrfe = tofl(sign, expo, mant, childRep);
		getrep(rf, sign, expo, mant, childRep, true);
		rlfe = tofl(sign, expo, mant, childRep);
	}

};

void testSpan()
{
	FloatRep P = { 1, 8, 7, 127, false };
	FloatRep C = { 0, 2, 2, 4, true };
	FloatRep L = { 0, 2, 1, 4, true };
	Span S(P, C, L);

	float maxerr = 0;

	while (1) {
		float l = randfloat();
		float r = randfloat();
		if (l > r)
			std::swap(l, r);

		float d = r - l;
		ASSERT_D(d > 0);
		float lr = l + DRand() * d;
		float rl = r - DRand() * d;

		// Could get numerical problems so detect and avoid
		if (!(r >= l) || !(lr > l && lr <= r) || !(rl >= l && rl < r))
			continue;

		float lrfe = 0, rlfe = 0;
		S.childSplits(l, r, lr, rl, lrfe, rlfe);

		float lre = l + d * lrfe;
		float rle = r - d * rlfe;

		float lw = lr - l;
		float rw = r - rl;
		ASSERT_D(lw > 0 && rw > 0);
		float lwe = lre - l;
		float rwe = r - rle;
		float lf = lwe / lw;
		float rf = rwe / rw;

		if (lf > maxerr || rf > maxerr) {
			std::cerr << l << " " << r << " " << lr << " " << rl << " " << lrfe << " " << rlfe << " " << lre << " " << rle << " " << lf << " " << rf << "\n";
			maxerr = std::max(lf, rf);
		}
	}
}

class Treelet
{
	std::vector<float> Coords;

};

int main()
{
	testSpan();
	//FloatRep F = { 0, 2, 2, 4, true };
	FloatRep F = { 1, 8, 7, 127, false };
	//test_getrep(F, F.zeroone);
	showvals(F);

	return 0;
}

#endif
