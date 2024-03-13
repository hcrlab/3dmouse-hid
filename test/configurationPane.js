import { Pane } from "tweakpane";

/**
 * Demo helper UI with sliders and checkboxes for tuning filtering parameters,
 * and interactive line plots for component values
 * @param container
 * @param {any?} status
 * @param {Object?} hostParam
 * @param filter input filtering object controls will modify
 * @param {Object?} filterParams
 * @param lastValue
 * @returns {{input: {}, folders: *[], pane: Pane, connectButton: *}}
 */
export function createConfigurationPane({
  container: container,
  status: status,
  hostParam: hostParam,
  filter: filter,
  filterParams: filterParams,
  lastValue: lastValue
}) {
  const pane = new Pane({ container: container });
  let connectButton = pane.addButton({
    title: "Connect to 3D Mouse",
    disabled: false,
  });
  if (status) {
    pane.addBinding(status, "message", {
      readonly: true,
      multiline: true,
      bufferSize: 5,
    });
    // Allow user to point to a roslibjs websocket of their choice
    if (hostParam) {
      pane.addBinding(hostParam, "host", { label: "ROS Host" });
    }
    for (let key of Object.keys(status)) {
      if (key === "message") continue;
      pane.addBinding(status, key, { readonly: true });
    }
  }
  let folders = [];
  let inputBindings = {};

  if (filter && filterParams) {
    const filteringFolder = pane.addFolder({
      title: "Filtering",
      disabled: true,
    });
    let translationScaleBinding = null;
    let rotationScaleBinding = null;
    // Having separate checkboxes for toggling translation and rotation allows the
    // user to specify a scale for each, then disable the component without losing the scale.
    filteringFolder
      .addBinding(filterParams, "translationEnabled", { label: "Translation" })
      .on("change", (e) => {
        filter.translationMultiplier = e.value ? filterParams["translationScale"] : 0;
        translationScaleBinding.disabled = !e.value
      });
    filteringFolder
      .addBinding(filterParams, "rotationEnabled", { label: "Rotation" })
      .on("change", (e) => {
        filter.rotationMultiplier = e.value ? filterParams["rotationScale"] : 0;
        rotationScaleBinding.disabled = !e.value
      });
    translationScaleBinding = filteringFolder
        .addBinding(filterParams, "translationScale", { label: "Translation Scale", min: 0.0, max: 3.0 })
        .on("change", (e) => {
          filter.translationMultiplier = e.value;
        });
    rotationScaleBinding = filteringFolder
        .addBinding(filterParams, "rotationScale", { label: "Rotation Scale", min: 0.0, max: 3.0 })
        .on("change", (e) => {
          filter.rotationMultiplier = e.value;
        });
    filteringFolder
      .addBinding(filterParams, "smoothing", { label:"Temporal Smoothing", min: 0.0, max: 1.0 })
      .on("change", (e) => {
        filter.smoothing = e.value;
      });
    filteringFolder
      .addBinding(filterParams, "softmaxTemperature", { label: "Softmax Temp", min: 0.01, max: 100 })
      .on("change", (e) => {
        filter.softmaxTemperature = e.value;
      });
    filteringFolder
      .addBinding(filterParams, "deadbandSize", { label: "Deadband Size", min: 0, max: 1.0 })
      .on("change", (e) => {
        filter.deadband = e.value;
      });
    filteringFolder
      .addBinding(filterParams, "deadbandWeight", { label: "Sensitivity (beta)", min: 0, max: 1.0 })
      .on("change", (e) => {
        filter.cubicDeadbandWeight = e.value;
      });
    folders["filtering"] = filteringFolder;
  }

  if (lastValue) {
    const graphFolder = pane.addFolder({
      title: "Filtered Graphs",
      disabled: true,
      expanded: false,
    });
    const GRAPH_OPTIONS = {
      readonly: true,
      view: "graph",
      min: -1,
      max: +1,
      interval: 10,
      bufferSize: 300,
    };
    for (let key of ["x", "y", "z", "roll", "pitch", "yaw"]) {
      graphFolder.addBinding(lastValue, key, GRAPH_OPTIONS);
    }
    const valueFolder = pane.addFolder({
      title: "Raw Values",
      disabled: true,
    });

    // Tweakpane doesn't have a readonly binding for Point3 unfortunately
    let xyzInput = valueFolder.addBinding(lastValue, "xyz", {
      readonly: false,
      x: { min: -1, max: 1 },
      y: { min: -1, max: 1 },
      z: { min: -1, max: 1 },
    });
    let rpyInput = valueFolder.addBinding(lastValue, "rpy", {
      readonly: false,
      x: { min: -1, max: 1 },
      y: { min: -1, max: 1 },
      z: { min: -1, max: 1 },
    });
    inputBindings = { xyz: xyzInput, rpy: rpyInput };
    valueFolder.addBinding(lastValue, "buttonsValue", { readonly: true });
    valueFolder.addBinding(lastValue, "t", { readonly: true });
    folders["graphs"] = graphFolder;
    folders["values"] = valueFolder;
  }

  return {
    pane: pane,
    connectButton: connectButton,
    folders: folders,
    input: inputBindings,
  };
}

export function updateLastValues(event, lastValue) {
  const values = event.detail.filteredInput;
  lastValue["x"] = values[0][0];
  lastValue["y"] = values[0][1];
  lastValue["z"] = values[0][2];
  lastValue["xyz"] = {
    x: lastValue["x"],
    y: lastValue["y"],
    z: lastValue["z"],
  };
  lastValue["roll"] = values[1][0];
  lastValue["pitch"] = values[1][1];
  lastValue["yaw"] = values[1][2];
  lastValue["rpy"] = {
    x: lastValue["roll"],
    y: lastValue["pitch"],
    z: lastValue["yaw"],
  };
  lastValue["buttonsValue"] = event.detail.buttonsValue;
  lastValue["t"] = event.detail.time;
}
