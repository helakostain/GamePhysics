#version 400

in vec4 ex_worldPosition;
in vec3 ex_worldNormal;

out vec4 out_color;

#define MAX_LIGHTS 100

struct Light {
    vec3 position;
    vec3 direction;
    float cutoff;
    vec3 lightColor;
    int lightType;
};

uniform Light lights[MAX_LIGHTS];
uniform int lightCount;

uniform vec3 ambientColor;


vec3 point_light(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightColor) {

    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(5.0 / dist, 0.0, 1.0);

    vec3 lightVector = normalize(lightPosition - vec3(ex_worldPosition));

    float dot_product = max(dot(lightVector, normalVector), 0.0);

    return dot_product * color * attenuation;
}

vec3 directional_light(vec3 color, vec3 normalVector, vec3 lightDirection, vec3 lightColor) {

    vec3 lightDir = normalize(-lightDirection);
    float dot_product = max(dot(lightDir, normalVector), 0.0);

    return dot_product * color;
}

vec3 spotlight(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightDirection, vec3 lightColor, float cutoff) {

    vec3 lightDir = normalize(lightPosition - worldPos);

    float theta = dot(lightDir, normalize(-lightDirection));
    if (theta <= cutoff) {
        return vec3(0.0, 0.0, 0.0);
    }

    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(5.0 / dist, 0.0, 1.0);

    vec3 lightVector = normalize(lightPosition - vec3(ex_worldPosition));

    float dot_product = max(dot(lightVector, normalVector), 0.0);

    return dot_product * color * attenuation;

}

void main() {

    vec3 diffuse = vec3(0.0, 0.0, 0.0);

    vec3 worldPos = vec3(ex_worldPosition);
    vec3 normalVector = normalize(ex_worldNormal);

    vec3 green = vec3(0.03, 0.30, 0.01);
    vec3 brown = vec3(0.49, 0.30, 0.12);
    vec3 color;

    float gCap = -3.0;
    float spread = 4;

    float b_ratio = (ex_worldPosition.y - gCap) / spread;
    b_ratio = min(1, max(0, b_ratio));
    float g_ratio = 1.0 - b_ratio;
    color = (b_ratio * brown) + (g_ratio * green);

    for (int index = 0; index < lightCount; ++index) {

        vec3 lightPosition = lights[index].position;
        vec3 lightColor = lights[index].lightColor;
        vec3 lightDirection = lights[index].direction;
        float cutoff = lights[index].cutoff;

        vec3 d;

        if (lights[index].lightType == 1) {
            d = point_light(color, worldPos, normalVector, lightPosition, lightColor);
        } else if (lights[index].lightType == 2) {
            d = directional_light(color, normalVector, lightDirection, lightColor);
        } else if (lights[index].lightType == 3) {
            d = spotlight(color, worldPos, normalVector, lightPosition, lightDirection, lightColor, cutoff);
        }

        diffuse = diffuse + d;
    }

    out_color = vec4(ambientColor + diffuse, 1.0);
}