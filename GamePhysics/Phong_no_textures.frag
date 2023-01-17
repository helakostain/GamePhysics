#version 400

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

uniform vec3 cameraPosition;
uniform vec3 ambientColor;
uniform vec3 objectColor;


in vec4 ex_worldPosition;
in vec3 ex_worldNormal;


out vec4 out_color;

vec3 point_light(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightColor) {

    const float specularStrength = 0.4;

    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(2.0 / dist, 0.0, 1.0);
    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 lightDir = normalize(lightPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);

    float dot_product = dot(lightDir, normalVector);
    vec3 diffuse = max(dot_product, 0.0) * (color+lightColor) * attenuation;

    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16);
    vec3 spec = specularStrength * specValue * lightColor;
    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }
    vec3 specular = attenuation * spec;

    return diffuse + specular;
}

vec3 spotlight(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightDirection, vec3 lightColor, float cutoff) {

    if(cutoff <=0.f){
        return vec3(1,1,1);
     }

    vec3 lightDir = normalize(lightPosition - worldPos);

    float theta = dot(lightDir, normalize(-lightDirection));
    if (theta <= cutoff) {
        return vec3(0.0, 0.0, 0.0);
    }

    const float specularStrength = 0.4;

    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(5.0 / dist, 0.0, 1.0);

    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);

    float dot_product = dot(lightDir, normalVector);
    vec3 diffuse = max(dot_product, 0.0) * (color+lightColor) * attenuation;

    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16);
    vec3 spec = specularStrength * specValue * lightColor;
    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }
    vec3 specular = attenuation * spec;

    return diffuse + specular;
}

vec3 directional_light(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightDirection, vec3 lightColor) {

    const float specularStrength = 0.4;

    vec3 lightDir = normalize(lightDirection);
    float dot_product = dot(lightDir, normalVector);
    vec3 diffuse = max(dot_product, 0.0) * lightColor;

    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);

    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16);
    vec3 spec = specularStrength * specValue * lightColor;
    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }

    return ((diffuse + spec) * color)*0.2; //for moon
    //return (diffuse + spec) * color; //for sun
}

void main () {

    vec3 fragColor = vec3(0.0, 0.0, 0.0);

    vec3 worldPos = ex_worldPosition.xyz / ex_worldPosition.w;
    vec3 normalVector = normalize(ex_worldNormal);

    vec3 color = vec3(0.5f,0.35f,0.05f);

    for (int index = 0; index < lightCount; ++index) {

        vec3 lightPosition = lights[index].position;
        vec3 lightColor = lights[index].lightColor;
        vec3 lightDirection = lights[index].direction;
        float cutoff = lights[index].cutoff;

        if (lights[index].lightType == 1) {
            fragColor += point_light(color, worldPos, normalVector, lightPosition, lightColor);
        } else if (lights[index].lightType == 2) {
            fragColor += directional_light(color, worldPos, normalVector, lightDirection, lightColor);
        } else if (lights[index].lightType == 3) {
            fragColor += spotlight(color, worldPos, normalVector, lightPosition, lightDirection, lightColor, cutoff);
        }
    }

    out_color = vec4(ambientColor + fragColor, 1.0);
}