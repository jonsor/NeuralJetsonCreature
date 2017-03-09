#include "stdafx.h"
#include "Cube.h"

Cube::Cube(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth) : position(position), color(color)
{
	// Cubes
	GLfloat cubeVertices[] = {
		//BACK
		-width, -height, -depth,  0.0f,  0.0f, -1.0f,
		width, -height, -depth,  0.0f,  0.0f, -1.0f,
		width,  height, -depth,  0.0f,  0.0f, -1.0f,
		width,  height, -depth,  0.0f,  0.0f, -1.0f,
		-width,  height, -depth,  0.0f,  0.0f, -1.0f,
		-width, -height, -depth,  0.0f,  0.0f, -1.0f,
		//FRONT
		-width, -height,  depth,  0.0f,  0.0f, 1.0f,
		width, -height,  depth,  0.0f,  0.0f, 1.0f,
		width,  height,  depth,  0.0f,  0.0f, 1.0f,
		width,  height,  depth,  0.0f,  0.0f, 1.0f,
		-width,  height,  depth,  0.0f,  0.0f, 1.0f,
		-width, -height,  depth,  0.0f,  0.0f, 1.0f,
		//LEFT SIDE
		-width,  height,  depth, -1.0f,  0.0f,  0.0f,
		-width,  height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height,  depth, -1.0f,  0.0f,  0.0f,
		-width,  height,  depth, -1.0f,  0.0f,  0.0f,
		//RIGHT SIDE
		width,  height,  depth,  1.0f,  0.0f,  0.0f,
		width,  height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height,  depth,  1.0f,  0.0f,  0.0f,
		width,  height,  depth,  1.0f,  0.0f,  0.0f,
		//BOTTOM
		-width, -height, -depth,  0.0f, -1.0f,  0.0f,
		width, -height, -depth,  0.0f, -1.0f,  0.0f,
		width, -height,  depth,  0.0f, -1.0f,  0.0f,
		width, -height,  depth,  0.0f, -1.0f,  0.0f,
		-width, -height,  depth,  0.0f, -1.0f,  0.0f,
		-width, -height, -depth,  0.0f, -1.0f,  0.0f,
		//TOP
		-width,  height, -depth,  0.0f,  1.0f,  0.0f,
		width,  height, -depth,  0.0f,  1.0f,  0.0f,
		width,  height,  depth,  0.0f,  1.0f,  0.0f,
		width,  height,  depth,  0.0f,  1.0f,  0.0f,
		-width,  height,  depth,  0.0f,  1.0f,  0.0f,
		-width,  height, -depth,  0.0f,  1.0f,  0.0f
	};

	angle = 0.0f;
	axisOfRotation = glm::vec3(1.0f, 1.0f, 1.0f);
	//CUBE STUFF
	GLuint cubeVBO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &cubeVBO);

	glBindVertexArray(cubeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind VAO

}

void Cube::render(Shader shader)
{
	shader.use();
	glBindVertexArray(cubeVAO);
	GLint objectColorLoc = glGetUniformLocation(shader.program, "objectColor");
	glUniform3f(objectColorLoc, color.x, color.y, color.z);

	glm::mat4 model;
	model = glm::translate(model, position);
	model = glm::rotate(model, angle, axisOfRotation);
	GLint modelLoc = glGetUniformLocation(shader.program, "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	//GLfloat angle = 20.0f * i;
	//model = glm::rotate(model, angle, glm::vec3(1.0f, 0.3f, 0.5f));
	glDrawArrays(GL_TRIANGLES, 0, 36);

	glBindVertexArray(0);
}

void Cube::setColor(glm::vec3 color)
{
	this->color = color;
}

glm::vec3 Cube::getColor()
{
	return color;
}

void Cube::setPosition(glm::vec3 position)
{
	//std::cout << position.y << std::endl;
	this->position = position;
}

glm::vec3 Cube::getPosition()
{
	return position;
}

void Cube::setRotation(GLfloat angle, glm::vec3 axisOfRotation)
{
	this->angle = angle;
	this->axisOfRotation = axisOfRotation;
}


Cube::~Cube()
{
}
