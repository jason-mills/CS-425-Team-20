//included to prevent cmd prompt from opening with program
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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

float angle = 0.0f;

float vertexColors[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
std::vector<stlTriangle> triangles;
int triangleCount = 0;

void initGL() 
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void display() 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    //// Render a color-cube consisting of 6 quads with different colors
    //glLoadIdentity();                 // Reset the model-view matrix
    //glTranslatef(1.5f, 0.0f, -7.0f);  // Move right and into the screen

    //glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    //// Top face (y = 1.0f)
    //// Define vertices in counter-clockwise (CCW) order with normal pointing out
    //glColor3f(0.0f, 1.0f, 0.0f);     // Green
    //glVertex3f(1.0f, 1.0f, -1.0f);
    //glVertex3f(-1.0f, 1.0f, -1.0f);
    //glVertex3f(-1.0f, 1.0f, 1.0f);
    //glVertex3f(1.0f, 1.0f, 1.0f);

    //// Bottom face (y = -1.0f)
    //glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    //glVertex3f(1.0f, -1.0f, 1.0f);
    //glVertex3f(-1.0f, -1.0f, 1.0f);
    //glVertex3f(-1.0f, -1.0f, -1.0f);
    //glVertex3f(1.0f, -1.0f, -1.0f);

    //// Front face  (z = 1.0f)
    //glColor3f(1.0f, 0.0f, 0.0f);     // Red
    //glVertex3f(1.0f, 1.0f, 1.0f);
    //glVertex3f(-1.0f, 1.0f, 1.0f);
    //glVertex3f(-1.0f, -1.0f, 1.0f);
    //glVertex3f(1.0f, -1.0f, 1.0f);

    //// Back face (z = -1.0f)
    //glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    //glVertex3f(1.0f, -1.0f, -1.0f);
    //glVertex3f(-1.0f, -1.0f, -1.0f);
    //glVertex3f(-1.0f, 1.0f, -1.0f);
    //glVertex3f(1.0f, 1.0f, -1.0f);

    //// Left face (x = -1.0f)
    //glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    //glVertex3f(-1.0f, 1.0f, 1.0f);
    //glVertex3f(-1.0f, 1.0f, -1.0f);
    //glVertex3f(-1.0f, -1.0f, -1.0f);
    //glVertex3f(-1.0f, -1.0f, 1.0f);

    //// Right face (x = 1.0f)
    //glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    //glVertex3f(1.0f, 1.0f, -1.0f);
    //glVertex3f(1.0f, 1.0f, 1.0f);
    //glVertex3f(1.0f, -1.0f, 1.0f);
    //glVertex3f(1.0f, -1.0f, -1.0f);
    //glEnd();  // End of drawing color-cube

    // Render a pyramid consists of 4 triangles
    //glLoadIdentity();                  // Reset the model-view matrix
    //glTranslatef(-1.5f, 0.0f, -6.0f);  // Move left and into the screen

    //glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles

    //for (int i = 0; i < triangleCount; i++)
    //{
    //    //glColor3f(vertexColors[i % 3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);     // Red
    //    glVertex3f(triangles[i].vertex1[0], triangles[i].vertex1[1], triangles[i].vertex1[2]);
    //    //glColor3f(vertexColors[i % 3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);     // Green
    //    glVertex3f(triangles[i].vertex2[0], triangles[i].vertex2[1], triangles[i].vertex2[2]);
    //    //glColor3f(vertexColors[i % 3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);
    //    glVertex3f(triangles[i].vertex3[0], triangles[i].vertex3[1], triangles[i].vertex3[2]);
    //}

    //glEnd();

    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

void idleRender() 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0f, 0.0f, 10.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f);
    glRotatef(angle, 0.0f, 1.0f, 0.0f);

    glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
    
    for (int i = 0; i < triangleCount; i++) 
    {
        glColor3f(vertexColors[i%3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);     // Red
        glVertex3f(triangles[i].vertex1[0], triangles[i].vertex1[1], triangles[i].vertex1[2]);
        glColor3f(vertexColors[i % 3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);     // Green
        glVertex3f(triangles[i].vertex2[0], triangles[i].vertex2[1], triangles[i].vertex2[2]);
        glColor3f(vertexColors[i % 3][0], vertexColors[i % 3][1], vertexColors[i % 3][2]);
        glVertex3f(triangles[i].vertex3[0], triangles[i].vertex3[1], triangles[i].vertex3[2]);
    }

    glEnd();   // Done drawing the pyramid

    angle += 0.3f;

    glutSwapBuffers();
}

void reshape(GLsizei width, GLsizei height) 
{
    if (height == 0)
        height = 1;
    GLfloat aspect = (GLfloat)width / (GLfloat)height;
    
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(20.0f, aspect, 1.0f, 1000.0f);
}

int main(int argc, char** argv)
{
    STL file("res/stl/Coral.stl");
    triangleCount = file.getTriangleCount();
    std::cout << triangleCount << std::endl;
    triangles = file.getTriangles();
    for (int i = 0; i < 10; i++) {
        std::cout << "Triangle #" << i + 1 << std::endl;
        std::cout << triangles[i].vertex1[0] << " " << triangles[i].vertex1[1] << " " << triangles[i].vertex1[2] << std::endl;
        std::cout << triangles[i].vertex2[0] << " " << triangles[i].vertex2[1] << " " << triangles[i].vertex2[2] << std::endl;
        std::cout << triangles[i].vertex3[0] << " " << triangles[i].vertex3[1] << " " << triangles[i].vertex3[2] << std::endl;

    }

    /*glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("Shape test");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idleRender);
    initGL();
    glutMainLoop();*/


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

    ////For stl example
    ///*
    // VertexArray va;
    // VertexBuffer vb(positions, triangleCount * 3 * sizeof(vertex));
    // VertexBufferLayout layout;
    // layout.Push<float>(3)
    // va.AddBuffer(bv, layout);
    // 
    //*/
    //
    //VertexArray va;
    //VertexBuffer vb(positions, 4 * 2 * sizeof(float));
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