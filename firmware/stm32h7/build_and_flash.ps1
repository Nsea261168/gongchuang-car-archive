# 向后兼容入口。推荐在 VS Code 中运行“Keil: Build + Flash”。
& (Join-Path $PSScriptRoot 'tools\keil.ps1') -Action BuildFlash
exit $LASTEXITCODE
