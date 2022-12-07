//included to prevent cmd prompt from opening with program
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>

#include "Renderer.h"
#include "IndexBuffer.h"
#include "VertexBuffer.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "Shader.h"
#include "STL.h"
#include "STL_Struct.h"

int main(void)
{
    STL stlfile("res/stl/SampleCube.stl");
    std::cout << "Header: " << stlfile.getName() << std::endl;
    std::cout << "Number of triangles: " << stlfile.getTriangleCount() << std::endl;

    std::vector<stlTriangles>

    //GLFWwindow* window;

    //if (!glfwInit())
    //    return -1;

    ////glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    ////glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    ////glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //window = glfwCreateWindow(640, 480, "File Name Here", NULL, NULL);
    //if (!window)
    //{
    //    glfwTerminate();
    //    return -1;
    //}

    //glfwMakeContextCurrent(window);

    //glfwSwapInterval(1);

    //if (glewInit() != GLEW_OK)
    //    return -1;

    //float positions[] = {
    //    -0.5f, -0.5f,
    //    0.5f, -0.5f,
    //    0.5f, 0.5f,
    //    -0.5f, 0.5f,
    //};

    //unsigned int indices[] = {
    //    0, 1, 2,
    //    2, 3, 0
    //};

    //VertexArray va;
    //VertexBuffer vb(positions, 4 * 2 * sizeof(float) * 2);
    //VertexBufferLayout layout;
    //layout.Push<float>(2);
    //va.AddBuffer(vb, layout);

    //IndexBuffer ib(indices, 6);

    //Shader shader("res/shaders/Basic.shader");
    //shader.Bind();
    //shader.SetUniform4f("uniformColor", 0.0f, 0.0f, 0.0f, 0.0f);

    //va.Unbind();
    //vb.Unbind();
    //ib.Unbind();
    //shader.Unbind();

    //Renderer renderer;

    //float r = 0.0f;
    //float increment = 0.05f;

    //while (!glfwWindowShouldClose(window))
    //{

    //    renderer.Clear();

    //    shader.Bind();
    //    shader.SetUniform4f("uniformColor", r, 0.0f, 0.0f, 0.0f);

    //    renderer.Draw(va, ib, shader);

    //    if (r > 1.0f)
    //        increment = -0.01f;
    //    else if (r < 0.0f)
    //        increment = 0.01f;

    //    r += increment;

    //    glfwSwapBuffers(window);
    //    glfwPollEvents();
    //}


    //glfwTerminate();

    //return 0;
}