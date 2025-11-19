#pragma once
#include "sim-tools/gui/includes-GL.hpp"
#include <unordered_map>
using std::unordered_map;

namespace sito {

class Shader {
    enum class ShaderType {
        VERTEX,
        FRAGMENT
    };
    inline static const map<ShaderType, string> shader_type_as_string{{ShaderType::VERTEX, "vertex"},
                                                                      {ShaderType::FRAGMENT, "fragment"}};
    unordered_map<string, int> uniform_location_cache;

  public:
    uint ID;
    void create(const string &vertex_name, const string &fragment_name, const path &shader_dir);
    void use() const;
    void set_int(const string &name, int val) { glUniform1i(get_uniform_location(name), val); }
    void set_bool(const string &name, bool val) { glUniform1i(get_uniform_location(name), val); }
    void set_float(const string &name, float val) { glUniform1f(get_uniform_location(name), val); }
    void set_vec3(const string &name, const glm::vec3 &val) { glUniform3fv(get_uniform_location(name), 1, &val[0]); }
    void set_vec4(const string &name, const glm::vec4 &val) { glUniform4fv(get_uniform_location(name), 1, &val[0]); }
    void set_mat4(const string &name, const glm::mat4 &val) {
        glUniformMatrix4fv(get_uniform_location(name), 1, GL_FALSE, &val[0][0]);
    }
    void set_mat3(const string &name, const glm::mat3 &val) {
        glUniformMatrix3fv(get_uniform_location(name), 1, GL_FALSE, &val[0][0]);
    }

  private:
    static string read_shader_source_code(const path &file_path);

    static uint create_and_compile_shader(ShaderType type, const path &source_path);
    static void check_compile_errors(uint shader, const string &type);

    int get_uniform_location(const string &name);
};

} // namespace sito