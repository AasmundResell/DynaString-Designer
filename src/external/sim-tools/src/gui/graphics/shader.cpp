#include "sim-tools/gui/graphics/shader.hpp"
#include <fstream>

namespace sito {

void Shader::create(const string &vertex_name, const string &fragment_name, const path &shader_dir) {
    assert(ID == 0 && uniform_location_cache.empty());

    const uint vertex = create_and_compile_shader(ShaderType::VERTEX, shader_dir / vertex_name);
    const uint fragment = create_and_compile_shader(ShaderType::FRAGMENT, shader_dir / fragment_name);

    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    check_compile_errors(ID, "program");
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}
void Shader::use() const {
    glUseProgram(ID);
}

int Shader::get_uniform_location(const string &name) {
    int location;
    if (uniform_location_cache.find(name) != uniform_location_cache.end()) {
        location = uniform_location_cache[name];
    } else {
        location = glGetUniformLocation(ID, name.c_str());
        if (location == -1) {
            assert(false);
            printf("Error: Uniform with name '%s' has value -1 (doesn't exist)\n", name.c_str());
            exit(EXIT_FAILURE);
        }
        uniform_location_cache[name] = location;
    }
    return location;
}

uint Shader::create_and_compile_shader(ShaderType type, const path &source_path) {

    const string source_code_str = read_shader_source_code(source_path);
    const char *source_code_raw = source_code_str.c_str();

    GLenum gl_shader_type;
    if (type == ShaderType::VERTEX) {
        gl_shader_type = GL_VERTEX_SHADER;
    } else {
        assert(type == ShaderType::FRAGMENT);
        gl_shader_type = GL_FRAGMENT_SHADER;
    }
    uint shader = glCreateShader(gl_shader_type);
    glShaderSource(shader, 1, &source_code_raw, NULL);
    glCompileShader(shader);
    check_compile_errors(shader, shader_type_as_string.at(type));
    return shader;
}

string Shader::read_shader_source_code(const path &file_path) {
    using namespace std;
    ifstream ist{file_path};
    if (!ist) {
        THROW_RUNTIME_ERROR("Error reading shader source with path " + file_path.string());
    }
    stringstream ss;
    ss << ist.rdbuf();
    return ss.str();
}

void Shader::check_compile_errors(uint shader, const string &type) {
    int success;
    constexpr uint BUFSIZE = 1024;
    char infoLog[BUFSIZE];
    if (type == "vertex" || type == "fragment") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, BUFSIZE, NULL, infoLog);
            THROW_RUNTIME_ERROR("Shader compilation of type " + type + " failed\n\n" + string(infoLog) +
                                "\n-----------------------------------------------------------\n");
        }
    } else if (type == "program") {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, BUFSIZE, NULL, infoLog);
            THROW_RUNTIME_ERROR("Program linking of type " + type + " failed\n\n" + string(infoLog) +
                                "\n-----------------------------------------------------------\n");
        }
    } else {
        assert(false);
    }
}
} // namespace sito