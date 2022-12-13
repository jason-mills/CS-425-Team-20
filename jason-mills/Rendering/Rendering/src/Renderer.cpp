#include "Renderer.h"
#include <iostream>

Renderer::Renderer() {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
}

void GLClearError()
{
    while (glGetError() != GL_NO_ERROR);
}

bool GLLogCall(const char* function, const char* file, int line)
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL Error] (" << error << "): " << function << " " << file << " " << line << std::endl;
        return false;
    }
    return true;
}

void Renderer::Clear() const
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

//void Renderer::Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader)
//{
//    shader.Bind();
//    va.Bind();
//    ib.Bind();
//    glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr);
//}

void Renderer::Draw(const VertexArray& va, const Shader& shader, GLsizei count)
{
    shader.Bind();
    va.Bind();
    glDrawArrays(GL_TRIANGLES, 0, count);
}

void Renderer::Draw3D(const VertexArray& va, const Shader& shader) 
{
    
    
}