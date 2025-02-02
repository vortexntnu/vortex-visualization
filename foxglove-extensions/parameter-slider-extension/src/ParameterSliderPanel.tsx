import {PanelExtensionContext, RenderState} from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState } from "react";
import ReactDOM from "react-dom";
import { Slider } from '@mui/material';
import type {NodeConfig, Parameter, ParameterProperties, ParameterValue, SetSrvParam} from "parameter_types";





let node: NodeConfig;
let paramNameList: string[];
let paramValList: ParameterValue[];

// input name of node along with optional properties of parameters (include '/' before node name to be compatible with service calls)
const nodeConfigListInit: NodeConfig[] = [
  {
    name: "/pcl_detector_node",
    parameters: [
      {
        name: "leaf_size",
        min_value: 0.0,
        max_value: 1.0,
        step: 0.01,
      },
      {
        name: "detector",
        dropdownOptions: ["euclidean", "dbscan"]
      },
      {
        name: "dbscan.epsilon",
        min_value: 0.0,
        max_value: 10.0,
        step: 0.1,
      },
      {
        name: "dbscan.min_points",
        min_value: 0,
        max_value: 100,
        step: 1,
      },
      {
        name: "euclidean.cluster_tolerance",
        min_value: 0.0,
        max_value: 10.0,
        step: 0.1,
      },
      {
        name: "euclidean.min_points",
        min_value: 0,
        max_value: 100,
        step: 1,
      },
    ],
  }
];




function ParameterSliderPanel({ context }: { context: PanelExtensionContext }): JSX.Element {


  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [status, setStatus] = useState<string | undefined>();

  const [paramList, setParamList] = useState<Array<Parameter>>();
  const [srvParamList, setSrvParamList] = useState<Array<SetSrvParam>>();
  const [nodeConfigList, setNodeList] = useState<NodeConfig[]>();
  const [colorScheme, setColorScheme] = useState<string>();
  const [bgColor, setBgColor] = useState("#d6d6d6");
  // const [loadButtonBgColor, setLoadButtonBgColor] = useState("#d6d6d6");



  useLayoutEffect( () => {

    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);
      updateNodeList();


      // Manage some styling for light and dark theme
      setColorScheme(renderState.colorScheme);
      if(renderState.colorScheme == "light") {
        setBgColor("#d6d6d6");
        // setLoadButtonBgColor("#d6d6d6");
      } else if (renderState.colorScheme == "dark") {
        setBgColor("#4d4d4d");
        // setLoadButtonBgColor("#4d4d4d");
      }
    };


    //If colorScheme changes, context.onRender() will change styling to match new color scheme
    context.watch("colorScheme");

  }, []);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);


  /**
   * converts string representation of a boolean to a boolean
   * @param stringValue "true" or "false"
   * @returns true or false
   */
  const stringToBoolean = (stringValue: string) => {
    switch(stringValue?.toLowerCase()?.trim()){
        case "true": return true;
        case "false": return false;
        default: return undefined;
    }
  }

  /**
   * determines if a string[] contains exclusively booleans
   * @param strArr string[] to check
   * @returns true if strArr only contains booleans, false otherwise
   */
  const isBooleanArr = (strArr: string[]) => {
    let bool: boolean = true;
    strArr.forEach(element => {
      // console.log(stringToBoolean(element));
      if(stringToBoolean(element) === undefined)
        bool = false;
    });

    return bool;
  }


    /**
   * Returns the string value of a paramter's value to be outputted on the screen
   * @param   param The parameter value that is converted to a string
   * @returns String representation of param
  */
  const getParameterValue = (param: ParameterValue) => {
    if(param === undefined) { return "undefined"; }
    switch(param.type) {
      case 1:  return param.bool_value.toString();
      case 2:  return param.integer_value.toString();
      case 3:  return param.double_value.toString();
      case 4:  return param.string_value;
      case 5:  return `[${param.byte_array_value.toString()}]`;
      case 6:  return `[${param.bool_array_value.toString()}]`;
      case 7:  return `[${param.integer_array_value.toString()}]`;
      case 8:  return `[${param.double_array_value.toString()}]`;
      case 9:  return `[${param.string_array_value.toString()}]`;
      default: return "error, invalid type...";
    }
  }

/**
//    * Updates the list of nodes upon initialization of the panel based on the content
//    */
const updateNodeList = () => {
  setStatus("retrieving nodes...")
  // context.callService?.("/rosapi/nodes", {})
  // .then((_values: unknown) =>{
    setNodeList(nodeConfigListInit);
    setStatus("nodes retrieved");
  }

/**
 * Retrieves a list of all parameters for the current node and their values
 */
const updateParamList = () =>{
  context.callService?.(node.name + "/list_parameters", {})
  .then((_value: unknown) => {
    paramNameList = (_value as any).result.names as string[];

    context.callService?.(node.name + "/get_parameters", {names: paramNameList})
    .then((_value: unknown) => {
      paramValList = (_value as any).values as ParameterValue[];

      let tempList:Array<Parameter> = [];
      for (let i = 0; i < paramNameList.length; i++) {
        tempList.push({name: paramNameList[i]!, value: paramValList[i]!});
      }
      if(tempList.length > 0)
        setParamList(tempList);

      if(paramNameList !== undefined) {
        setSrvParamList(new Array(paramList?.length));
      }
    })
    .catch(() => {setStatus("error, failed to retrieve parameter values")});
  })
  .catch(() => {setStatus("error, failed to retrieve parameter list")});
}

/**
 * Sets new values to all parameters with an inputted new value
 * Calls updateParamList() to 'refresh' the screen and display the new parameter values
 */
const setParam = () => {
  setStatus("setting parameters...");

  let tempList: SetSrvParam[] = srvParamList!;

  for(let i: number = 0; i < tempList.length; i++) {
    if(tempList[i] == null) {
      tempList.splice(i, 1);
      i = -1;
    }
  }

  setSrvParamList(tempList);
  context.callService?.(node.name + "/set_parameters", {parameters: srvParamList})
  .then(() => {
    updateParamList();
    setStatus("parameters set");
  })
  .catch((error: Error) => {
    updateParamList();
    setStatus("Error: " + JSON.stringify(error));
  });
}

/**
 * Update the list of Parameters with new values to be set
 * @param val The new value to be set
 * @param name The name of the parameter that will be set to 'val'
 */
const updateSrvParamList = (name: string, val: string) => {
  let idx: number = paramNameList?.indexOf(name)!;
  let tempList: SetSrvParam[] = srvParamList!;
  let tempValList: string[] = [];

  if(val === "") {
    const emptyP: SetSrvParam = {};
    tempList[idx] = emptyP;
  } else {
    let ssp: SetSrvParam = {};
    let valStrArr: string[] = [];
    switch (paramList![idx]?.value.type!) {
      case 1:
        ssp = { name: name, value: { type: 1, bool_value: stringToBoolean(val) }};
        break;

      case 2:
        ssp = { name: name, value: { type: 2, integer_value: +val }};
        break;

      case 3:
        ssp = { name: name, value: { type: 3, double_value: +val }};
        break;

      case 4:
        ssp = { name: name, value: { type: 4, string_value: val }};
        break;

      // TODO: Implement format for byte arrays
      case 5:
        //ssp = { name: name, value: { type: 5, byte_array_value: val as unknown as number[] }};
        break;

      case 6:
        valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
        if(isBooleanArr(valStrArr)) {
          let valBoolArr: boolean[] = valStrArr.map((element) => {
            if(element == "true")
              return true;
            return false;
          });
          ssp = { name: name, value: { type: 6, bool_array_value: valBoolArr}};
        }
        break;

      case 7:
      valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
        ssp = { name: name, value: { type: 7, integer_array_value: valStrArr.map(Number) }};
        break;

      case 8:
        valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
        ssp = { name: name, value: { type: 8, double_array_value: valStrArr.map(Number) }};
        break;

      case 9:
        val.replace(" ", "");
        if(val.charAt(0) == '[' && val.charAt(val.length - 1) == ']')
          val = val.substring(1, val.length - 1);
        valStrArr = val.split(",");
        ssp = { name: name, value: { type: 9, string_array_value: valStrArr }};
        break;

      default: ssp = {}; break;
    }
    tempList[idx] = ssp;
    setSrvParamList(tempList)
  }

  paramValList.forEach(element => {
    tempValList.push(getParameterValue(element));
  });
}

/**
 * Returns the properties of a parameter
 * @param   param The parameter that is being checked
 * @returns The properties of the parameter
 */
const getPropertiesOfParam = (param: Parameter) => {

  // Check if node is undefined
  if (!node) {
      // Handle the case where node is not available
      return null;
  }

  // Find the NodeConfig in nodeList
  let nodeConfig: NodeConfig | undefined = nodeConfigList?.find(n => n.name === node.name);

  // Check if nodeConfig is undefined
  if (!nodeConfig) {
      // Handle the case where nodeConfig is not found
      return null;
  }

  // Access parameters array and find the specific parameter
  let parameterProperties: ParameterProperties | undefined = nodeConfig.parameters?.find(p => p.name === param.name);

  // Check if parameterProperties is undefined
  if (!parameterProperties) {
      // Handle the case where parameterProperties is not available
      return null;
  }

  return parameterProperties;
}


/**
 * Creates a dropdown input box if param is a boolean or dropdownOptions are specified, creates a slider if param is int or double or inputbox otherwise
 * @param   param The parameter that an input box is being created for
 * @returns A dropdown if param.value.type == 1 or dropdownOptions are defined, slider if param.value.type==2 or 3
 */
const createInputBox = (param: Parameter) => {
  let parameterProperties: ParameterProperties | null = getPropertiesOfParam(param);
  if(!parameterProperties){
    return;
  }
  if(parameterProperties?.dropdownOptions){
    return (
      <select
        value={getParameterValue(param.value)}
        style={dropDownStyle}
        onChange={(event) => handleSliderandDropdownChange(param.name, event.target.value)}
      >
         <option selected hidden></option>
      {parameterProperties.dropdownOptions.map((option, index) => (
        <option key={index} value={option}>
          {option}
        </option>
      ))}
      </select>
    );
  }


  if (param.value.type === 1) {
    return (
      <select
        value={getParameterValue(param.value)}
        style={dropDownStyle}
        onChange={(event) => handleSliderandDropdownChange(param.name, event.target.value)}
      >
        <option selected hidden></option>
          <option>true</option>
          <option>false</option>
      </select>
    );
  } else if (param.value.type === 2) {
    return (
      <Slider
        style={{ color: colorScheme === 'dark' ? '#f7f7f7' : '#333333' }}
        min={parameterProperties?.min_value ?? 0}
        max={parameterProperties?.max_value ?? 50}
        step={parameterProperties?.step ?? 1}
        valueLabelDisplay="auto"
        value={parseInt(getParameterValue(param.value), 10)}
        onChangeCommitted={(_, value) => handleSliderandDropdownChange(param.name, value.toString())}
      />
    );
  }
  else if (param.value.type === 3) {
    return (
      <Slider
        style={{ color: colorScheme === 'dark' ? '#f7f7f7' : '#333333'}}
        min={parameterProperties?.min_value ?? 0}
        max={parameterProperties?.max_value ?? 5}
        step={parameterProperties?.step ?? 0.1}
        valueLabelDisplay="auto"
        value={parseFloat(getParameterValue(param.value))}
        onChangeCommitted={(_, value) => handleSliderandDropdownChange(param.name, value.toString())}
      />
    );
  }
  return (
    <input
      style={inputStyle}
      placeholder={getParameterValue(param.value)}
      onChange={(event) => updateSrvParamList(param.name, event.target.value)}
    />
  );
};

/**
 * Updates the parameter list with the new value of the parameter and invokes setParam()
 * @param name The name of the parameter that will be set to 'value'
 * @param value The new value of the parameter
 */
const handleSliderandDropdownChange = (name: string, value: string) => {
  updateSrvParamList(name, value.toString());
  setParam();
};

/**
 * Creates a inputbox regardless of param type
 * @param   param The parameter that an input box is being created for
 * @returns inputbox
 */
const createInputOnlyBox = (param: Parameter) => {
  let parameterProperties: ParameterProperties | null = getPropertiesOfParam(param);
  if(!parameterProperties){
    return;
  }
  let value = getParameterValue(param.value);
  if (param.value.type === 3) {
    value = parseFloat(value).toFixed(1);
  }
  return (
    <input
      style={inputStyle}
      placeholder={value}
      onChange={(event) => updateSrvParamList(param.name, event.target.value)}
    />
  );
};

  /**
   * loads parameter values from a YAML file and sets all new values
   * @param files the YAML file to be uploaded
   */
  // const loadFile = (files: FileList | null) => {
  //   if(files !== null) {
  //     files[0]?.text()
  //     .then((value: string) => {
  //       value = value.replaceAll(/[^\S\r\n]/gi, "");
  //       value = value.replace(node + ":\n", "");
  //       value = value.replace("ros__parameters:\n", "");

  //       let params: string[] = value.split("\n");
  //       for(let i = 0; i < params.length; i++) {

  //         if(params[i]!.charAt(0) != '-' && params[i]!.charAt(params[i]!.length - 1) != ':') {
  //           let temp: string[]= params[i]!.split(":");
  //           updateSrvParamList(temp[0]!, temp[1]!);

  //         } else if(params[i]!.charAt(params[i]!.length - 1) == ':') {
  //           let tempName: string = params[i]!.replace(":", "").trim();
  //           let tempVal: string = "";

  //           while(i + 1 < params.length && params[++i]!.charAt(0) == '-') {
  //             tempVal = tempVal.concat(params[i]!.replace("-", "").trim() + ",");
  //           }

  //           i--;
  //           tempVal = tempVal.substring(0, tempVal.length-1);
  //           updateSrvParamList(tempName, tempVal);
  //         }
  //       }
  //       setParam();
  //     })
  //     .catch((error: Error) => {
  //       console.log(error)
  //     });
  //   }
  // }

///////////////////////////////////////////////////////////////////
//////////////////////// PANEL LAYOUT /////////////////////////////
///////////////////////////////////////////////////////////////////


//////////////////////// CSS STYLING //////////////////////////////

let setButtonStyle = {};
// let loadButtonStyle = {};
let dropDownStyle = {};
let inputStyle = {};

if(colorScheme == "light") {

  setButtonStyle = {

    fontSize: "1rem",
    backgroundColor: bgColor,
    border: bgColor + " solid",
    margin: "36px 12px 36px 0px",
    padding: "8px",
    borderRadius: "4px",
    color: "#333333",
    fontWeight: "500",

  };

  // loadButtonStyle = {

  //   fontSize: "1rem",
  //   backgroundColor: loadButtonBgColor,
  //   border: loadButtonBgColor + " solid",
  //   margin: "36px 0px 36px 12px",
  //   padding: "8px",
  //   borderRadius: "4px",
  //   color: "#333333",
  //   fontWeight: "500",

  // };

  dropDownStyle = {

    fontSize: "1rem",
    padding: "3px",
    flex: 1,
    backgroundColor: "#f7f7f7",
    color: "#333333",
    borderRadius: "3px",
    marginRight: "3px"
  };

  inputStyle = {

    fontSize: "1rem",
    padding: "3px",
    backgroundColor: "#f7f7f7",
    border: "1px solid #333333",
    borderRadius: "3px",
    width: "75px"
  };

} else if(colorScheme == "dark") {

  setButtonStyle = {

    fontSize: "1rem",
    backgroundColor: bgColor,
    border: bgColor + " solid",
    margin: "36px 12px 36px 0px",
    padding: "8px",
    borderRadius: "4px",
    color: "#f7f7f7",
    fontWeight: "500",

  };

  // loadButtonStyle = {

  //   fontSize: "1rem",
  //   backgroundColor: loadButtonBgColor,
  //   border: loadButtonBgColor + " solid",
  //   margin: "36px 0px 36px 12px",
  //   padding: "8px",
  //   borderRadius: "4px",
  //   color: "#f7f7f7",
  //   fontWeight: "500",

  // };

  dropDownStyle = {

    fontSize: "1rem",
    padding: "3px",
    flex: 1,
    backgroundColor: "#4d4d4d",
    color: "#f7f7f7",
    borderRadius: "3px",

  };


  inputStyle = {

    fontSize: "1rem",
    padding: "3px",
    backgroundColor: "#4d4d4d",
    color: "#f7f7f7",
    border: "1px solid #4d4d4d",
    borderRadius: "3px",
    width: "75px"
  };

}

const labelStyle = {
  fontSize: "1.35rem",
  paddingRight: "12px",
  marginBottom: "10px",
  fontWeight: "500",
}

const statusStyle = {
  fontSize: "0.8rem",
  padding: "5px",
  borderTop: "0.5px solid",
}

const footerStyle = {
  backgroundColor: "#F8F8F8",
  borderTop: "1px solid #E7E7E7",
  textAlign: "center",
  padding: "20px",
  position: "fixed",
  left: "0",
  bottom: "0",
  height: "60px",
  width: "100%"
};
footerStyle;

///////////////////////////////////////////////////////////////////

///////////////////////// HTML PANEL //////////////////////////////

return (
  <body>
  <div style={{ padding: "1rem",
                scrollBehavior: "smooth",
                maxHeight:"calc(100% - 25px)",
                overflowY: "scroll",
                fontFamily: "helvetica",
                fontSize: "1rem",
                }}>
    <h1>ROS2 Parameter Extension</h1>
    <label style={labelStyle}>Node:</label>
    <select
      value={node?.name}
      onChange={(event) => {
        // Find the selected node in the nodeList based on its name
        const selectedNode = nodeConfigList?.find(n => n.name === event.target.value);

        // Check if a node is found
        if (selectedNode) {
          // Update the node variable with the selectedNode
          node = selectedNode;

          // Trigger the updateParamList function
          updateParamList();
        }
      }}
      style={dropDownStyle}
      >
      <option selected hidden>Select a Node</option>
      {(nodeConfigList ?? []).map((node) => (
        <option key={node?.name} value={node?.name}>{node?.name}</option>
      ))}
    </select>

    <form>
      <button
        style={setButtonStyle}
        onMouseEnter={() => setBgColor("#8f8f8f")}
        onMouseLeave={() => colorScheme == "dark" ? setBgColor("#4d4d4d"): setBgColor("#d6d6d6")}
        onClick={setParam}
        type="reset">
          Set Parameters
      </button>

      {/* <label
        style={loadButtonStyle}
        onMouseEnter={() => setLoadButtonBgColor("#8f8f8f")}
        onMouseLeave={() => colorScheme == "dark" ? setLoadButtonBgColor("#4d4d4d"): setLoadButtonBgColor("#d6d6d6")}
        >
        <input type="file" style={{display: "none"}} onChange={(event) => {loadFile(event.target.files)}}/>
          Load
      </label> */}
      <br/>

      <div style={{ display: "grid", gridTemplateColumns: "1.1fr 0.5fr 1.0fr"}}>
        <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px", textDecoration: "bold"}}>Parameter</b>
        {/* <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px", textDecoration: "bold"}}>Type</b> */}
        <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px", textDecoration: "bold"}}>Value</b>
        <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px", textDecoration: "bold"}}>New Value</b>
      </div>
        {(paramList ?? []).map((result,index) => {
          // Get the properties of the parameter
          let parameterProperties: ParameterProperties | null = getPropertiesOfParam(result);

          // Check if parameterProperties is undefined
          if (!parameterProperties) {
            // If properties are not available, skip rendering
            return null;
          }
          let bc = index % 2 === 0 ? "#808080" : "white";
          if (colorScheme === "dark") {
            bc = index % 2 === 0 ? "#1a1a1a" : "#333333";
          }
          return (
            <div style={{ display: "grid", gridTemplateColumns: "1.1fr 0.5fr 1.0fr", backgroundColor: bc, padding: "4px"}}>
              <div style={{ margin: "0px" }}>{result.name}:</div>
              {/* <div style={{ margin: "0px"}}>{getType(result)}</div> */}
              <div style={{ margin: "0px"}}>{createInputOnlyBox(result)}</div>
              <div style={{ margin: "0px"}}>{createInputBox(result)}</div>
            </div>
          );
        })}
    </form>
  </div>
  <div style={{left: "0px", bottom: "0px", height: "25px", width: "100%", position: "sticky"}}>
    <p style={statusStyle}>status: {status} {nodeConfigList?.toString()} </p>
  </div>
  </body>
);

///////////////////////////////////////////////////////////////////

}

export function initParameterSliderPanel(context: PanelExtensionContext) {
ReactDOM.render(<ParameterSliderPanel context={context} />, context.panelElement);
}
