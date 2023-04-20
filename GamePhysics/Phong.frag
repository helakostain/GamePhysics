#version 400
#define MAX_LIGHTS 100

struct Light {
    vec3 position;
    vec3 direction;
    float cutoff;
    vec3 lightColor;
    int lightType;
};

struct Material
{
  vec3 diffuse;
  vec3 specular;
  vec3 ambient;
  float shininess;
};

uniform Light lights[MAX_LIGHTS];
uniform int lightCount;
uniform vec3 cameraPosition;
uniform vec3 ambientColor;
uniform vec3 objectColor;
uniform sampler2D textureUnitID;
uniform sampler2D normalUnitID;
uniform Material material;
in vec4 ex_worldPosition;
in vec3 ex_worldNormal;
in vec2 uv;
out vec4 out_color;

vec3 point_light(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightColor) {
    const float specularStrength = 0.4;
    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(2.0 / dist, 0.0, 1.0); //weaking light over distance
    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 lightDir = normalize(lightPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);
    float dot_product = dot(lightDir, normalVector);
    float diffuse = max(dot_product, 0.0)  * attenuation; //part of object where light lands
    //float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16); 
    //vec3 spec = specularStrength * specValue * lightColor; //shiniest part of light on object

    vec3 diff =  (lightColor) * (diffuse * material.diffuse);
    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), material.shininess);
    vec3 spec = specularStrength * (specValue * material.specular) * lightColor;

    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }
    vec3 specular = attenuation * spec;

    return (diffuse + specular) * color;
}

vec3 spotlight(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightPosition, vec3 lightDirection, vec3 lightColor, float cutoff) {

    vec3 lightDir = normalize(lightPosition - worldPos);

    float theta = dot(lightDir, normalize(-lightDirection));
    if (theta <= cutoff) {
        return vec3(0.0, 0.0, 0.0);
    }

    const float specularStrength = 0.4;

    float dist = length(lightPosition - worldPos);
    float attenuation = clamp(2.0 / dist, 0.0, 1.0);

    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);

    float dot_product = dot(lightDir, normalVector);
    float diffuse = max(dot_product, 0.0) * attenuation;

    //float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16);
    //vec3 spec = specularStrength * specValue * lightColor;

    vec3 diff =  (lightColor/10) * (diffuse * material.diffuse);
    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), material.shininess);
    vec3 spec = specularStrength * (specValue * material.specular) * (lightColor/10);

    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }
    vec3 specular = attenuation * spec;

    return (diffuse + specular) * color;
}

vec3 directional_light(vec3 color, vec3 worldPos, vec3 normalVector, vec3 lightDirection, vec3 lightColor) {


    
    const float specularStrength = 0.4;

    vec3 lightDir = normalize(lightDirection); //norm
    float dot_product = dot(lightDir, normalVector);
    float diffuse = max(dot_product, 0.0);
    
    vec3 viewDir = normalize(cameraPosition - worldPos);
    vec3 reflectionDir = reflect(-lightDir, normalVector);
    
    vec3 diff =  (lightColor) * (diffuse * material.diffuse);
    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), material.shininess);
    vec3 spec = specularStrength * (specValue * material.specular) * lightColor;
    if (dot_product < 0.0) {
        spec = vec3(0.0);
    }

    //return ((diffuse + spec) * color)*0.2; //for moon
    return (diff + spec) * color; //for sun
}

void main () {

    vec3 fragColor = vec3(0.0, 0.0, 0.0);

    vec3 worldPos = ex_worldPosition.xyz / ex_worldPosition.w;
    vec3 normalVector = normalize(ex_worldNormal);

    vec4 tex = texture(textureUnitID, uv);
    //vec4 tex = texture(textureUnitID, uv * 0);

    vec3 color = vec3(tex.x, tex.y, tex.z);
    //vec3 color = vec3(uv.x, uv.y, 0.0f);

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

    vec3 out_color2 = ambientColor + fragColor;
    out_color2*= 1.5f;
    out_color2 = out_color2 / (1.0f + out_color2);
    out_color = vec4(pow(out_color2, vec3(1.0f/2.4f)), 1);
    //TODO: gamma correction test (need polishing ALOT)
   // float gamma = 1.2;
   // out_color = vec4(pow(ambientColor, vec3(1.0/gamma)) + fragColor, 1);
}