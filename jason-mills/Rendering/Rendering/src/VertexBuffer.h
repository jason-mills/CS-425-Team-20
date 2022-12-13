#pragma once
#include<vector>

class VertexBuffer
{
private:
	unsigned int rendererId;
public:
	VertexBuffer(const void* data, unsigned int size);
	VertexBuffer(std::vector<float> data);
	~VertexBuffer();

	void Bind() const;
	void Unbind() const;
};