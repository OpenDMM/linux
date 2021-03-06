/* LibTomMath, multiple-precision integer library -- Tom St Denis
 *
 * LibTomMath is a library that provides multiple-precision
 * integer arithmetic as well as number theoretic functionality.
 *
 * The library was designed directly after the MPI library by
 * Michael Fromberger but has been written from scratch with
 * additional optimizations in place.
 *
 * The library is free for all purposes without any express
 * guarantee it works.
 *
 * Tom St Denis, tomstdenis@iahu.ca, http://math.libtomcrypt.org
 */
#include <tommath.h>

/* determines if an integers is divisible by one 
 * of the first PRIME_SIZE primes or not
 *
 * sets result to 0 if not, 1 if yes
 */
int mp_prime_is_divisible(mp_int * a, int *result)
{
	int err, ix;
	mp_digit res;

	/* default to not */
	*result = 0;

	for (ix = 0; ix < PRIME_SIZE; ix++) {
		/* what is a mod __prime_tab[ix] */
		if ((err = mp_mod_d(a, __prime_tab[ix], &res)) != MP_OKAY) {
			return err;
		}

		/* is the residue zero? */
		if (res == 0) {
			*result = 1;
			return MP_OKAY;
		}
	}

	return MP_OKAY;
}
