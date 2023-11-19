import { ExtensionContext } from "@foxglove/studio";
import { initParameterSliderPanel as initParameterSliderPanel } from "./ParameterSliderPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Parameter Sliders", initPanel: initParameterSliderPanel });
}
