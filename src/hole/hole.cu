#include "hole.hpp"
#include "misc/includes.hpp"

#include "sim-tools/arena/arena.hpp"

// Needed due to assertion
int Hole::get_increment_index_pipe_node_to_hole_segment_initial(uint ih, const scalar s_i, const byte *buf) const {
    assert(ih >= 0 && ih < N_hole + 1); // check bounds

    const ArrayView<scalar> s = get_field<HoleField::s>(buf);

    if (s_i < s[ih]) {
        return -1;
    } else if (s_i <= s[ih + 1]) {
        return 0;
    } else {
        assert(s_i > s[ih + 1]);
        return 1;
    }
}

void Hole::initalize_pipe_nodes_to_hole_segments(const uint N, const Config &config, uint *indices, const ArrayView<scalar> S, const byte *buf) const {
    const uint Ne_hole = N_hole + 1; //Pluss two ghost nodes
    const scalar L = config.conf_stat.L;
    const scalar L_feed = config.conf_stat.L_string_depth_initial;

    for (uint i = 0; i < N; i++) {
        int hi = indices[i];
        assert(hi == 1 || hi == 0); /*Initial value of hi*/
        const scalar s = -L + L_feed + S[i];
        int search_dir = get_increment_index_pipe_node_to_hole_segment_initial(hi, s, buf);
        if (search_dir == 0) {
            continue;
        } else if (search_dir == -1) {
            indices[i] = 0; // All nodes outside domain set to 0
        } else {
            /*Search forwards*/
            while (search_dir != 0) {
                assert(search_dir == 1);
                hi++;
                assert(hi < (int)Ne_hole);
                if (hi >= (int)Ne_hole) {
                    throw runtime_error("Hole index (" + to_string(hi) + ") >= num hole elements (" +
                                        to_string(Ne_hole) + ") found for node i=" + to_string(i));
                }
                search_dir = get_increment_index_pipe_node_to_hole_segment_initial(hi, s, buf);
            }
            indices[i] = min(hi, (int)Ne_hole - 1);
        }
    }
}
