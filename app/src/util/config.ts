import Ajv from "ajv";
import uncheckedConfig from "../../config/main.yaml";
import type { AppConfig } from "../types/config";
import { schema } from "../types/config";

const ajv = new Ajv();
const validate = ajv.compile(schema);

let config: AppConfig;

if (validate(uncheckedConfig)) {
  config = uncheckedConfig;
  console.log("Configuration validated successfully!");
} else {
  console.error("Configuration validation failed:");
  console.error(validate.errors);
  // Handle the error appropriately, e.g., throw an error, use default config, etc.
  throw new Error("Invalid configuration");
}

export default config;
