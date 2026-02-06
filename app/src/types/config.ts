import type { JSONSchemaType } from "ajv";

export interface AppConfig {
  rosIp: string;
}

export const schema: JSONSchemaType<AppConfig> = {
  type: "object",
  properties: {
    rosIp: { type: "string" },
  },
  required: ["rosIp"],
  additionalProperties: false,
};
