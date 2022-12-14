#pragma once

#include<GL/glew.h>
#include"VertexArray.h"
#include "IndexBuffer.h"
#include "Shader.h"

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
    x;\
    ASSERT(GLLogCall(#x, __FILE__, __LINE__));

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);

//singleton???
class Renderer
{
private:

public:
    Renderer();
    void Clear() const;
    //void Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader);
    void Draw(const VertexArray& va, const Shader& shader, GLsizei count);
    void Draw3D(const VertexArray& va, const Shader& shader);
};