#version 450

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

uniform sampler2D textureUnitID;
uniform sampler2D normalUnitID;

uniform int intensity=1;

in vec3 _normal;

in VS_OUT {
    vec3 FragPos;
    vec2 TexCoords;
    vec3 NormalPos;
    mat3 TBN;
} fs_in;


out vec4 out_color;


vec3 point_light(vec3 color, vec3 lightColor, vec3 normal, vec3 TangentLightPos,  vec3 TangentViewPos,  vec3 TangentFragPos)
{
    vec3 ambient = 0.1 * color;

    // diffuse
    vec3 lightDir = normalize(TangentLightPos - TangentFragPos);

    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * (lightColor + color);

    // specular
    vec3 viewDir = normalize(TangentViewPos - TangentFragPos);

    //vec3 reflectDir = reflect(-lightDir, normal);

    vec3 halfwayDir = normalize(lightDir + viewDir);  

    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);

    vec3 specular = vec3(0.2) * spec;

    return ambient + diffuse + specular;
}

vec3 directional_light(vec3 color, vec3 lightColor, vec3 normal, vec3 lightDirection, vec3 TangentLightPos,  vec3 TangentViewPos,  vec3 TangentFragPos)
{
    vec3 ambient = 0.1 * color;
    const float specularStrength = 0.4;

    vec3 lightDir = normalize(lightDirection);
    float dot_product = dot(lightDir, normal);
    vec3 diffuse = max(dot_product, 0.0) * lightColor;

    vec3 viewDir = normalize(TangentViewPos - TangentFragPos);
    vec3 reflectionDir = reflect(-lightDir, normal);

    float specValue = pow(max(dot(viewDir, reflectionDir), 0.0), 16);
    vec3 spec = specularStrength * specValue * lightColor;

    if (dot_product < 0.0)
    {
        spec = vec3(0.0);
    }

    return (diffuse + spec) * color;
}

vec3 spotlight(vec3 color, vec3 lightColor, vec3 normal, vec3 lightDirection, vec3 TangentLightPos,  vec3 TangentViewPos,  vec3 TangentFragPos, float cutoff) 
{
    vec3 ambient = 0.1 * color;
    // diffuse
    vec3 lightDir = normalize(TangentLightPos - TangentFragPos);

    float theta = dot(lightDir, normalize(-lightDirection));
    if (theta <= cutoff) {
        return vec3(0.0, 0.0, 0.0);
    }

    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * (lightColor + color);

    // specular
    vec3 viewDir = normalize(TangentViewPos - TangentFragPos);

    //vec3 reflectDir = reflect(-lightDir, normal);

    vec3 halfwayDir = normalize(lightDir + viewDir);  

    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);

    vec3 specular = vec3(0.2) * spec;

    return ambient + diffuse + specular;
}

void main () 
{

    vec4 resultColor = vec4(0.0);

    for (int i = 0; i < lightCount; i++)
    {
        vec3 TangentLightPos = lights[i].position;
        vec3 TangentViewPos = cameraPosition;
        vec3 TangentFragPos =  fs_in.FragPos;

        vec3 lightDirection = lights[i].direction;
        vec3 lightColor = lights[i].lightColor;
        float cutoff = lights[i].cutoff;

        // obtain normal from normal map in range [0,1]
        vec3 normal = texture(normalUnitID, fs_in.TexCoords).rgb;
        normal = 2.0f * normal - 1.0f;
        normal = normalize(normal * vec3(1,1,1));
        normal = normalize(fs_in.TBN * normal);

        // get diffuse color
        vec3 color = texture(textureUnitID, fs_in.TexCoords).rgb;
        // ambient

        if (lights[i].lightType == 1)
        {
            resultColor += vec4(point_light(color, lightColor, normal, TangentLightPos, TangentViewPos, TangentFragPos),1.0);
        } else if (lights[i].lightType == 2) 
        {
            resultColor += vec4(directional_light(color, lightColor, normal, lightDirection, TangentLightPos, TangentViewPos, TangentFragPos),1.0);
        } else if (lights[i].lightType == 3)
        {
            resultColor += vec4(spotlight(color, lightColor, normal, lightDirection, TangentLightPos, TangentViewPos, TangentFragPos, cutoff),1.0);
        }
    }

    out_color = resultColor;
}