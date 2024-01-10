# parameter-slider-extension

## _A Foxglove Studio Extension_

[Foxglove Studio](https://github.com/foxglove/studio) allows developers to create extensions, or custom code that is loaded and executed inside the Foxglove Studio application. This can be used to add custom panels. Extensions are authored in TypeScript using the `@foxglove/studio` SDK.

This extension requires a foxglove bridge to work. Source your ros2 workspace and run the `foxglove bridge` executable to launch the `foxglove bridge`.

```sh
sudo ros2 run foxglove_bridge foxglove_bridge
```

## Installation setup

Extension development uses the `npm` package manager to install development dependencies and run build scripts.

First make sure you have `node.js` installed on your system. 

```sh
sudo apt install nodejs
```
 
If installation was successful you should now have node.js installed
```sh
node -v
```
Should now return `v20.8.0` or a newer version. If you still have an older version you need to install a newer one using NVM. First run
```sh
sudo apt install curl 
curl https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash 
```
The nvm installer script creates an environment entry to the login script of the current user. You can either log out and log in again to load the environment or execute the below command to do the same.

```sh
source ~/.bashrc
nvm install node 
```
You should now have the latest version of node

To install necessary extension dependencies, run `npm` from the root of the extension package `/foxglove-extensions/parameter-slider-extension`.

```sh
npm install
```


## Package

Extensions are packaged into `.foxe` files. These files contain the metadata (package.json) and the build code for the extension.


```sh
npm run package
```

This command will package the extension into a `.foxe` file in the local directory. Right click the generated `.foxe` file in your file explorer and select `Open With Other Application`, then choose the `Foxglove Studio` option. 

This launces `Foxglove Studio` and installs the extension contained inside the `.foxe` to your local `Foxglove Studio` directory. The `parameter-slider-extension` is now availabe in the panels tab under the name `Parameter Sliders [local]` every time you launch `Foxglove Studio`.


## Develop 


To develop custom properties for your own nodes simply input the name of your node and the properties of the parameters you want displayed `Parameter Sliders [local]` panel in `Foxglove Studio`. 


### Data format

Your custom configuration has to be inputted as a `NodeConfig` type declared in the `/src/types/index.d.ts` file.

You input your own `NodeConfig` by adding it to the declared value of `NodeConfigListInit` in the  `/src/ParameterSliderPanel.tsx` file. Use the already existing `NodeConfig` types as a guide on how to expand upon the `NodeConfigListInit` list.


```ts
    export type NodeConfig = {
        name: string;
        parameters?: ParameterProperties[]; 

    }
    
    export type ParameterProperties = {
        name: string;
        dropdownOptions?: (number | string)[];
        min_value?: number
        max_value?: number
        step?: number
        
    }
```

Fields marked with `?` implies that the property is optional and not required. 

### Node name

The only required field in a `NodeConfig` type is the name of the node you want displayed. Note that you have to include a `/` as a prefix to the name of your node in order for the service calls used in the extension to be valid. 

### Parameters

If you want a parameter to be displayed in the exstension panel you have to specify the parameter as the name of a `ParameterProperties` type. Parameter names not included as a `ParameterProperties` will not be displayed in the exstension panel, so that unnecessary or constant parameters are not displayed.

### Properties of a Parameter

All the properties of a `ParameterProperties` type are optional and will be set to a default value if undefined. 

Defining `dropdownOptions` is useful when dealing with a parameter that only has a few valid values. If `dropdownOptions` it will create a custum dropdownbox where easily choose between the valid values of the parameter. Not that if a parameter is of `boolean` type it will automatically be assigned a dropdownbox.

The remaining optinal properties of the parameter are used when initializing the slider for the parameter if the parameter is of the `boolean` type. Since they are optional they will be assigned to default values if not specified.


## TODO
The template for loading and setting parameters from a `.yaml` file is currently commented out in the `/src/ParameterSliderPanel.tsx` file. Develop this at your own free will `:)`


