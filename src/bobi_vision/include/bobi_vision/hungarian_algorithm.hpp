#ifndef BOBI_HUNGARIAN_ALGORITHM_HPP
#define BOBI_HUNGARIAN_ALGORITHM_HPP

// based on https://github.com/mcximing/hungarian-algorithm-cpp

#include <numeric>
#include <vector>

namespace bobi {
    namespace algo {
        class HungarianAlgorithm {

        public:
            HungarianAlgorithm() {}
            ~HungarianAlgorithm() {}

            double solve(const std::vector<std::vector<double>>& costs, std::vector<int>& assigned_idcs)
            {
                size_t num_rows = costs.size();
                size_t num_cols = costs[0].size();

                std::vector<double> cost_flattened(num_cols * num_rows);
                double cost = 0.0;

                for (size_t i = 0; i < num_rows; i++) {
                    for (size_t j = 0; j < num_cols; j++) {
                        cost_flattened[i + num_rows * j] = costs[i][j];
                    }
                }

                return opt_assignment(assigned_idcs, cost_flattened, num_rows, num_cols);
            }

            double opt_assignment(std::vector<int>& assigned_idcs, std::vector<double> costs, size_t num_rows, size_t num_cols)
            {
                size_t num_elements = num_rows * num_cols;
                double cost = 0;

                assigned_idcs.resize(num_rows, -1);

                std::vector<bool> marked_cols(num_cols);
                std::vector<bool> marked_rows(num_rows);
                std::vector<bool> star_mat(num_elements);
                std::vector<bool> star_mat_hat(num_elements);
                std::vector<bool> prime_mat(num_elements);

                if (num_rows <= num_cols) {

                    for (size_t m = 0; m < num_rows; ++m) {
                        double min_val = std::numeric_limits<double>::infinity();
                        for (size_t n = 0; n < num_cols; ++n) {
                            if (costs[m + num_rows * n] < min_val) {
                                min_val = costs[m + num_rows * n];
                            }
                        }

                        for (size_t n = 0; n < num_cols; ++n) {
                            costs[m + num_rows * n] -= min_val;
                        }
                    }

                    for (size_t m = 0; m < num_rows; ++m) {
                        for (size_t n = 0; n < num_cols; ++n) {
                            if (std::abs(costs[m + num_rows * n]) < std::numeric_limits<double>::epsilon()) {
                                if (!marked_cols[n]) {
                                    star_mat[m + num_rows * n] = true;
                                    marked_cols[n] = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                else {
                    for (size_t n = 0; n < num_cols; n++) {
                        double min_val = std::numeric_limits<double>::infinity();
                        for (size_t m = 0; m < num_rows; ++m) {
                            if (costs[m + num_rows * n] < min_val) {
                                min_val = costs[m + num_rows * n];
                            }
                        }

                        for (size_t m = 0; m < num_rows; ++m) {
                            costs[m + num_rows * n] -= min_val;
                        }
                    }

                    for (size_t n = 0; n < num_cols; ++n) {
                        for (size_t m = 0; m < num_rows; ++m) {
                            if (std::abs(costs[m + num_rows * n]) < std::numeric_limits<double>::epsilon()) {
                                if (!marked_rows[m]) {
                                    star_mat[m + num_rows * n] = true;
                                    marked_cols[n] = true;
                                    marked_rows[m] = true;
                                    break;
                                }
                            }
                        }
                    }

                    for (size_t m = 0; m < num_rows; ++m) {
                        marked_rows[m] = false;
                    }
                }

                _step2b(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);

                cost = _compute_cost(assigned_idcs, costs, num_rows);
                return cost;
            }

            void init_assignment_vec(std::vector<int>& assigned_idcs, std::vector<bool>& star_mat, size_t num_rows, size_t num_cols)
            {
                for (size_t m = 0; m < num_rows; ++m) {
                    for (size_t n = 0; n < num_cols; ++n) {
                        if (star_mat[m + num_rows * n]) {
                            assigned_idcs[m] = n;
                            break;
                        }
                    }
                }
            }

            double _compute_cost(std::vector<int>& assigned_idcs, const std::vector<double>& costs, size_t num_rows)
            {
                double cost = 0;
                for (size_t m = 0; m < num_rows; m++) {
                    size_t n = assigned_idcs[m];
                    if (n >= 0) {
                        cost += costs[m + num_rows * n];
                    }
                }
                return cost;
            }

            void _step2a(std::vector<int>& assigned_idcs, std::vector<double>& costs, std::vector<bool>& star_mat, std::vector<bool>& star_mat_hat, std::vector<bool>& prime_mat, std::vector<bool>& marked_cols, std::vector<bool>& marked_rows, size_t num_rows, size_t num_cols)
            {
                for (size_t n = 0; n < num_cols; ++n) {
                    for (size_t m = 0; m < num_rows; ++m) {
                        if (star_mat[m + num_rows * n]) {
                            marked_cols[n] = true;
                            break;
                        }
                    }
                }

                _step2b(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);
            }

            void _step2b(std::vector<int>& assigned_idcs, std::vector<double>& costs, std::vector<bool>& star_mat, std::vector<bool>& star_mat_hat, std::vector<bool>& prime_mat, std::vector<bool>& marked_cols, std::vector<bool>& marked_rows, size_t num_rows, size_t num_cols)
            {
                int col, nOfCoveredColumns;

                size_t num_marked_cols = 0;
                for (size_t n = 0; n < num_cols; ++n) {
                    if (marked_cols[n]) {
                        ++num_marked_cols;
                    }
                }

                if (num_marked_cols == std::min(num_rows, num_cols)) {
                    // finalize
                    init_assignment_vec(assigned_idcs, star_mat, num_rows, num_cols);
                }
                else {
                    _step3(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);
                }
            }

            void _step3(std::vector<int>& assigned_idcs, std::vector<double>& costs, std::vector<bool>& star_mat, std::vector<bool>& star_mat_hat, std::vector<bool>& prime_mat, std::vector<bool>& marked_cols, std::vector<bool>& marked_rows, size_t num_rows, size_t num_cols)
            {
                size_t s_col;
                bool is_marked = true;
                while (is_marked) {
                    is_marked = false;
                    for (size_t n = 0; n < num_cols; ++n)
                        if (!marked_cols[n])
                            for (size_t m = 0; m < num_rows; ++m)
                                if ((!marked_rows[m]) && (fabs(costs[m + num_rows * n]) < std::numeric_limits<double>::epsilon())) {
                                    prime_mat[m + num_rows * n] = true;

                                    for (s_col = 0; s_col < num_cols; ++s_col)
                                        if (star_mat[m + num_rows * s_col])
                                            break;

                                    if (s_col == num_cols) {
                                        _step4(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols, m, n);
                                        return;
                                    }
                                    else {
                                        marked_rows[m] = true;
                                        marked_cols[s_col] = false;
                                        is_marked = true;
                                        break;
                                    }
                                }
                }

                _step5(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);
            }

            void _step4(std::vector<int>& assigned_idcs, std::vector<double>& costs, std::vector<bool>& star_mat, std::vector<bool>& star_mat_hat, std::vector<bool>& prime_mat, std::vector<bool>& marked_cols, std::vector<bool>& marked_rows, size_t num_rows, size_t num_cols, size_t row, size_t col)
            {
                std::copy(star_mat.begin(), star_mat.end(), star_mat_hat.begin());
                star_mat_hat[row + num_rows * col] = true;

                size_t s_col = col;
                size_t s_row;
                for (s_row = 0; s_row < num_rows; ++s_row) {
                    if (star_mat[s_row + num_rows * s_col]) {
                        break;
                    }
                }

                while (s_row < num_rows) {
                    star_mat_hat[s_row + num_rows * s_col] = false;

                    size_t p_row = s_row;
                    size_t p_col;
                    for (p_col = 0; p_col < num_cols; p_col++) {
                        if (prime_mat[p_row + num_rows * p_col]) {
                            break;
                        }
                    }

                    star_mat_hat[p_row + num_rows * p_col] = true;

                    s_col = p_col;
                    for (s_row = 0; s_row < num_rows; s_row++) {
                        if (star_mat[s_row + num_rows * s_col]) {
                            break;
                        }
                    }
                }

                for (size_t i = 0; i < num_cols * num_rows; ++i) {
                    prime_mat[i] = false;
                    star_mat[i] = star_mat_hat[i];
                }

                for (size_t m = 0; m < num_rows; ++m) {
                    marked_rows[m] = false;
                }

                _step2a(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);
            }

            void _step5(std::vector<int>& assigned_idcs, std::vector<double>& costs, std::vector<bool>& star_mat, std::vector<bool>& star_mat_hat, std::vector<bool>& prime_mat, std::vector<bool>& marked_cols, std::vector<bool>& marked_rows, size_t num_rows, size_t num_cols)
            {
                double min_val = std::numeric_limits<double>::infinity();
                for (size_t m = 0; m < num_rows; m++) {
                    if (!marked_rows[m]) {
                        for (size_t n = 0; n < num_cols; n++) {
                            if (!marked_cols[n]) {
                                if (costs[m + num_rows * n] < min_val)
                                    min_val = costs[m + num_rows * n];
                            }
                        }
                    }
                }

                for (size_t m = 0; m < num_rows; m++) {
                    if (marked_rows[m]) {
                        for (size_t n = 0; n < num_cols; n++) {
                            costs[m + num_rows * n] += min_val;
                        }
                    }
                }

                for (size_t n = 0; n < num_cols; n++) {
                    if (!marked_cols[n]) {
                        for (size_t m = 0; m < num_rows; m++) {
                            costs[m + num_rows * n] -= min_val;
                        }
                    }
                }

                _step3(assigned_idcs, costs, star_mat, star_mat_hat, prime_mat, marked_cols, marked_rows, num_rows, num_cols);
            }
        };

    } // namespace algo
} // namespace bobi

#endif