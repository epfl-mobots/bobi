#ifndef RANDOM_GENERATOR_HPP
#define RANDOM_GENERATOR_HPP

#include <iostream>
#include <random>
#include <tools/random/rand_utils.hpp>
#include <type_traits>

namespace bobi {
    namespace tools {

        namespace _gen {
            static thread_local std::mt19937 gen(randutils::auto_seed_128{}.base());
        }

        template <typename T,
            typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
        T random_in_range(T lower_bound, T upper_bound)
        {
            std::uniform_real_distribution<T> distribution(lower_bound, upper_bound);
            return distribution(_gen::gen);
        }

        template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
        T random_in_range(T lower_bound, T upper_bound)
        {
            std::uniform_int_distribution<T> distribution(lower_bound, upper_bound);
            return distribution(_gen::gen);
        }

        template <typename T,
            typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
        T normal_random(T mean, T sigma)
        {
            std::normal_distribution<T> distribution(mean, sigma);
            return distribution(_gen::gen);
        }

        int8_t random_sgn();

    } // namespace tools
} // namespace bobi

#endif
