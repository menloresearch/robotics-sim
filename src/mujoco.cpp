#include <emscripten.h>
#include <emscripten/html5.h>
#include <mujoco/mujoco.h>

// Globals
mjModel *m = NULL;
mjData *d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// Main render function (called each frame)
void render() {
  // Step simulation
  mj_step(m, d);

  // Update viewport size
  mjrRect viewport = {0, 0, 0, 0};
  viewport.width = EM_ASM_INT(return canvas.width);
  viewport.height = EM_ASM_INT(return canvas.height);

  // Render scene
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);
}

// Main function
int main() {
  // Load model
  char error[1000];

  const char *str = R"(
  <mujoco>
    <worldbody>
      <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
      <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
      <body pos="0 0 1">
        <joint type="free"/>
        <geom type="box" size=".1 .2 .3" rgba="0 .9 0 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  mjSpec *parse = mj_parseXMLString(str, NULL, error, 1000);
  m = mj_compile(parse, NULL);
  d = mj_makeData(m);

  // Init camera and scene
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Position camera
  cam.distance = 2.0;
  cam.azimuth = 90.0;

  // Start render loop
  emscripten_set_main_loop(render, 0, 1);

  return 0;
}
