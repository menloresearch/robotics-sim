#include <GLES3/gl3.h>
#include <emscripten.h>
#include <emscripten/html5.h>
#include <stdio.h>

// WebGL variables
GLuint shaderProgram;
GLuint VBO, VAO;

// Simple shader setup
void setupShaders() {
  // Vertex shader
  const char *vertexShaderSource = "attribute vec3 aPos;\n"
                                   "attribute vec3 aColor;\n"
                                   "varying vec3 vColor;\n"
                                   "void main() {\n"
                                   "   gl_Position = vec4(aPos, 1.0);\n"
                                   "   vColor = aColor;\n"
                                   "}\n";

  // Fragment shader
  const char *fragmentShaderSource = "precision mediump float;\n"
                                     "varying vec3 vColor;\n"
                                     "void main() {\n"
                                     "   gl_FragColor = vec4(vColor, 1.0);\n"
                                     "}\n";

  // Compile vertex shader
  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
  glCompileShader(vertexShader);

  // Compile fragment shader
  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragmentShader);

  // Create shader program
  shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  // Clean up shaders
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  // Create buffers
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  printf("Shaders set up successfully\n");
}

// Render function
void render() {
  // Clear screen with blue color
  glClearColor(0.2f, 0.3f, 0.8f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Use shader
  glUseProgram(shaderProgram);

  // Bind buffers
  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);

  // Triangle vertices (position + color)
  float vertices[] = {
      // Position (XYZ)    // Color (RGB)
      -0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, // Bottom left - red
      0.5f,  -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, // Bottom right - green
      0.0f,  0.5f,  0.0f, 0.0f, 0.0f, 1.0f  // Top - blue
  };

  // Upload data
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Set position attribute (first 3 floats)
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  // Set color attribute (next 3 floats)
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // Draw triangle
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // Check for errors
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    printf("OpenGL error: 0x%x\n", err);
  }
}

// Main function
int main() {
  printf("Starting WebGL example\n");

  // Initialize WebGL
  EmscriptenWebGLContextAttributes attrs;
  emscripten_webgl_init_context_attributes(&attrs);
  attrs.majorVersion = 2;
  attrs.minorVersion = 0;

  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx =
      emscripten_webgl_create_context("#canvas", &attrs);
  if (ctx <= 0) {
    printf("Failed to create WebGL context: %d\n", ctx);
    return 1;
  }
  emscripten_webgl_make_context_current(ctx);
  printf("WebGL context created\n");

  // Setup shaders
  setupShaders();

  // Start render loop
  printf("Starting render loop\n");
  emscripten_set_main_loop(render, 0, 1);

  return 0;
}
