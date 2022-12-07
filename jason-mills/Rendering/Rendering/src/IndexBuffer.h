#pragma once

class IndexBuffer
{
private:
	unsigned int rendererId;
	unsigned int indexCount;
public:
	IndexBuffer(const unsigned int* data, unsigned int indexCount);
	~IndexBuffer();

	void Bind() const;
	void Unbind() const;

	inline unsigned int GetCount() const { return indexCount; }
};

