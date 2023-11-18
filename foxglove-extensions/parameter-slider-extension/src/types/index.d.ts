declare module "parameter_types" {

    export type NodeConfig = {
        name?: string;
        parameters?: ParameterProperties[]; 

    }

    export type Parameter = {
        name: string;
        value: ParameterValue;       
    }

    export type ParameterProperties = {
        name?: string;
        dropdownOptions?: (number | string)[];
        min_value?: number
        max_value?: number
        
    }

    export type ParameterValue = {
        type: number;
        bool_value: boolean;
        integer_value: int;
        double_value: float;
        string_value: string;
        byte_array_value: number[];
        bool_array_value: boolean[];
        integer_array_value: number[];
        double_array_value: number[];
        string_array_value: string[];
    }

    export type SetSrvParam = { 
        name?: string;
        value?: {
            type: number;
            bool_value?: boolean;
            integer_value?: int;
            double_value?: float;
            string_value?: string;
            byte_array_value?: number[];
            bool_array_value?: boolean[];
            integer_array_value?: number[];
            double_array_value?: number[];
            string_array_value?: string[];
        }
    }



}