#include "VertexBuffer.h"
#include "Renderer.h"

VertexBuffer::VertexBuffer(const void* data, unsigned int size)
{
	glGenBuffers(1, &rendererId);
	glBindBuffer(GL_ARRAY_BUFFER, rendererId);
	glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
}
VertexBuffer::~VertexBuffer()
{
	glDeleteBuffers(1, &rendererId);
}

void VertexBuffer::Bind() const
{
	glBindBuffer(GL_ARRAY_BUFFER, rendererId);
}
void VertexBuffer::Unbind() const
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}