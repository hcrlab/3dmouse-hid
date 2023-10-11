import {Pane} from 'tweakpane';

export function createConfigurationPane({
                                            container: container,
                                            status: status,
                                            filter: filter,
                                            filterParams: filterParams,
                                            lastValue: lastValue
                                        }) {
    const pane = new Pane({container: container});
    let connectButton = pane.addButton({title: "Connect 3D Mouse", disabled: false})
    if (status) {
        pane.addBinding(status, 'message', {readonly: true, multiline: true, bufferSize: 5})
        for (let key of Object.keys(status)) {
            if (key === "message") continue;
            pane.addBinding(status, key, {readonly: true})
        }
    }
    let folders = []
    let inputBindings = {}

    if (filter && filterParams) {
        const filteringFolder = pane.addFolder({
            title: 'Filtering',
            disabled: true
        });
        filteringFolder.addBinding(filterParams, "translationEnabled").on("change", e => {
            filter.translationMultiplier = e.value ? 1. : 0.;
        })
        filteringFolder.addBinding(filterParams, "rotationEnabled").on("change", e => {
            filter.rotationMultiplier = e.value ? 1. : 0.;
        })
        filteringFolder.addBinding(
            filterParams, 'smoothing',
            {min: 0.00, max: 1.0}
        ).on("change", e => {
            filter.smoothing = e.value
        })
        filteringFolder.addBinding(
            filterParams, 'softmaxTemperature',
            {min: 0.01, max: 100}
        ).on("change", e => {
            filter.softmaxTemperature = e.value
        })
        filteringFolder.addBinding(
            filterParams, 'deadbandSize',
            {min: 0., max: 1.0}
        ).on("change", e => {
            filter.deadband = e.value
        })
        filteringFolder.addBinding(
            filterParams, 'deadbandWeight',
            {min: 0, max: 1.0}
        ).on("change", e => {
            filter.cubicDeadbandWeight = e.value
        })
        folders["filtering"] = filteringFolder
    }

    if (lastValue) {
        const graphFolder = pane.addFolder({
            title: 'Graphs',
            disabled: true,
            expanded: false
        });
        const GRAPH_OPTIONS = {
            readonly: true,
            view: 'graph',
            min: -1,
            max: +1,
            interval: 10,
            bufferSize: 300
        }
        for (let key of ["x", "y", "z", "roll", "pitch", "yaw"]) {
            graphFolder.addBinding(lastValue, key, GRAPH_OPTIONS)
        }
        const valueFolder = pane.addFolder({
            title: 'Values',
            disabled: true
        });

        // Tweakpane doesn't have a readonly binding for Point3 unfortunately
        let xyzInput = valueFolder.addBinding(lastValue, "xyz", {
            readonly: false,
            x: {min: -1, max: 1},
            y: {min: -1, max: 1},
            z: {min: -1, max: 1}
        })
        let rpyInput = valueFolder.addBinding(lastValue, "rpy", {
            readonly: false,
            x: {min: -1, max: 1},
            y: {min: -1, max: 1},
            z: {min: -1, max: 1}
        })
        inputBindings = {xyz: xyzInput, rpy: rpyInput}
        valueFolder.addBinding(lastValue, 'buttonsValue', {readonly: true})
        valueFolder.addBinding(lastValue, 't', {readonly: true})
        folders["graphs"] = graphFolder
        folders["values"] = valueFolder
    }

    return {pane: pane, connectButton: connectButton, folders: folders, input: inputBindings}
}

export function updateLastValues(event, lastValue) {
    const values = event.detail.filteredInput
    lastValue["x"] = values[0][0]
    lastValue["y"] = values[0][1]
    lastValue["z"] = values[0][2]
    lastValue["xyz"] = {x: lastValue["x"], y: lastValue["y"], z: lastValue["z"]}
    lastValue["roll"] = values[1][0]
    lastValue["pitch"] = values[1][1]
    lastValue["yaw"] = values[1][2]
    lastValue["rpy"] = {x: lastValue["roll"], y: lastValue["pitch"], z: lastValue["yaw"]}
    lastValue["buttonsValue"] = event.detail.buttonsValue
    lastValue["t"] = event.detail.time
}