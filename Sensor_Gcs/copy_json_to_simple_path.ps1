# Serial Studio JSON 파일 경로 문제 해결 스크립트
# 한글이 포함된 복잡한 경로에서 단순한 경로로 JSON 파일을 복사합니다

$sourceFile = "2026 AAS CANSAT.json"
$targetDir = "C:\Cansat"
$targetFile = "$targetDir\2026_AAS_CANSAT.json"

# 현재 스크립트 위치에서 소스 파일 찾기
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourcePath = Join-Path $scriptPath $sourceFile

# 소스 파일 존재 확인
if (-not (Test-Path $sourcePath)) {
    Write-Host "오류: $sourceFile 파일을 찾을 수 없습니다." -ForegroundColor Red
    Write-Host "경로: $sourcePath" -ForegroundColor Yellow
    exit 1
}

# 대상 디렉토리 생성
if (-not (Test-Path $targetDir)) {
    Write-Host "디렉토리 생성: $targetDir" -ForegroundColor Cyan
    New-Item -ItemType Directory -Path $targetDir -Force | Out-Null
}

# 파일 복사
try {
    Copy-Item -Path $sourcePath -Destination $targetFile -Force
    Write-Host "성공: JSON 파일이 복사되었습니다!" -ForegroundColor Green
    Write-Host "원본: $sourcePath" -ForegroundColor Gray
    Write-Host "대상: $targetFile" -ForegroundColor Gray
    Write-Host ""
    Write-Host "Serial Studio에서 다음 경로의 파일을 선택하세요:" -ForegroundColor Yellow
    Write-Host $targetFile -ForegroundColor White
} catch {
    Write-Host "오류: 파일 복사 실패 - $_" -ForegroundColor Red
    exit 1
}
