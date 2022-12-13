#include "Renderer.h"
#include "VertexBufferLayout.h"
#include "VertexArray.h"

VertexArray::VertexArray()
{
	glGenVertexArrays(1, &rendererId);
}
VertexArray::~VertexArray()
{
	glDeleteVertexArrays(1, &rendererId);
}

void VertexArray::AddBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout)
{
	Bind();
	vb.Bind();
	const std::vector<VertexBufferElement> elements = layout.GetElements();
	unsigned int offset = 0;
	for (int i = 0; i < elements.size(); i++)
	{
		VertexBufferElement element = elements[i];
		glEnableVertexAttribArray(i);
		glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(), (const void*)offset);
		offset += element.count * VertexBufferElement::GetSizeOfType(element.type);
	}
}

void  VertexArray::Bind() const
{
	glBindVertexArray(rendererId);
}
void  VertexArray::Unbind() const
{
	glBindVertexArray(0);
}