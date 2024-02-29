#version 410 core

in vec3 fPosition;
in vec3 fNormal;
flat in vec3 fNormalFlat;
in vec2 fTexCoords;
uniform bool flatShading;

out vec4 fColor;
vec3 normalToUse;

//matrices
uniform mat4 model;
uniform mat4 view;
uniform mat3 normalMatrix;
//lighting
uniform vec3 lightDir;
uniform vec3 lightColor;
// textures
uniform sampler2D diffuseTexture;
uniform sampler2D specularTexture;
uniform vec3 lightPos;
uniform vec3 cameraPos;

uniform sampler2D shadowMap;
in vec4 fragPosLightSpace;

//components
vec3 ambient;
float ambientStrength = 0.2f;
vec3 diffuse;
vec3 specular;
float specularStrength = 0.5f;
float shininess = 35.0f;
float constant = 1.0f;
float linear = 0.0045f;
float quadratic = 0.0075f;

vec3 computeDirLight(float shadow)
{
    vec3 lightDirN = vec3(normalize(view * vec4(lightDir, 0.0f)));
    vec4 fPosEye = view * model * vec4(fPosition, 1.0f);
    vec3 viewDirN = normalize(-fPosEye.xyz);
    vec3 halfVector = normalize(lightDirN + viewDirN);
    vec3 normalEye = normalize(normalMatrix * normalToUse);
    float specCoeff = pow(max(dot(normalEye, halfVector), 0.0f), shininess);
    
    vec3 ambient = ambientStrength * lightColor;
    vec3 diffuse = max(dot(normalEye, lightDirN), 0.0f) * lightColor;
    vec3 specular = specularStrength * specCoeff * lightColor;

    return (ambient + (1.0f - shadow) * diffuse) * texture(diffuseTexture, fTexCoords).rgb + (1.0f - shadow) * specular * texture(specularTexture, fTexCoords).rgb;
}

vec3 computePointLight()
{
    vec4 fPosEye = view * model * vec4(fPosition, 1.0f);
    vec4 lightPosEye = view * vec4(lightPos, 1.0f);
    vec3 lightDirN = normalize(lightPosEye.xyz - fPosEye.xyz);
    vec3 viewDirN = normalize(cameraPos - fPosEye.xyz);
    vec3 halfVector = normalize(lightDirN + viewDirN);
    vec3 normalEye = normalize(normalMatrix * normalToUse);

    float dist = length(lightPosEye.xyz - fPosEye.xyz);
    float att = 1.0 / (constant + linear * dist + quadratic * (dist * dist));

    float specCoeff = pow(max(dot(normalEye, halfVector), 0.0f), shininess);
    
    vec3 ambient = att * ambientStrength * lightColor;
    vec3 diffuse = att * max(dot(normalEye, lightDirN), 0.0f) * lightColor;
    vec3 specular = att * specularStrength * specCoeff * lightColor;

    return (ambient + diffuse) * texture(diffuseTexture, fTexCoords).rgb + specular * texture(specularTexture, fTexCoords).rgb;
}

float computeFog()
{
 float fogDensity = 0.01f;
 vec4 fPosEye = view * model * vec4(fPosition, 1.0f);
 float fragmentDistance = length(fPosEye);
 float fogFactor = exp(-pow(fragmentDistance * fogDensity, 2));

 return clamp(fogFactor, 0.0f, 1.0f);
}

float computeShadow()
{
    float bias = 0.005f;

    vec3 normalizedCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    normalizedCoords = normalizedCoords * 0.5f + 0.5f;

    if (normalizedCoords.z > 1.0f)
        return 0.0f;

    float closestDepth = texture(shadowMap, normalizedCoords.xy).r;
    float currentDepth = normalizedCoords.z;
    float shadow = currentDepth - bias > closestDepth ? 1.0f : 0.0f;

    return shadow;
}

void main() 
{
    if (flatShading) {
        normalToUse = fNormalFlat; // Use flat normal
    } else {
        normalToUse = fNormal; // Use interpolated normal
    }
    float shadow = computeShadow();

    //compute final vertex color
    vec3 color = min(computeDirLight(shadow) + computePointLight(), 1.0f);
    float fogFactor = computeFog();
    vec4 fogColor = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    fColor = mix(vec4(fogColor.rgb, 1.0), vec4(color, 1.0), fogFactor);
}
