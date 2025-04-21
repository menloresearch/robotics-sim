#include "mujoco/mujoco.h"
#include <stdio.h>

char error[1000];

int main() {
  printf("Mujoco version: %s\n\n", mj_versionString());

  const char *xml = R"(
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
  mjSpec *parse = mj_parseXMLString(xml, NULL, error, 1000);
  mjModel *model = mj_compile(parse, NULL);

  if (!model) {
    printf("%s\n", error);
    return 1;
  }

  mjData *data = mj_makeData(model);

  int tmp = 0;

  while (data->time < 10) {
    if (tmp != data->ncon) {
      printf("Number of detected contacts %d", data->ncon);
      tmp = data->ncon;
    }
    mj_step(model, data);
  }

  mj_deleteData(data);
  mj_deleteModel(model);

  return 0;
}
