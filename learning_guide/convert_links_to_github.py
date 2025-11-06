#!/usr/bin/env python3
"""
마크다운 파일의 상대경로 링크를 GitHub 절대경로로 변환하는 스크립트

사용법:
    python convert_links_to_github.py
"""

import os
import re
from pathlib import Path

# GitHub 저장소 기본 URL
GITHUB_BASE = "https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide"

def convert_relative_to_github(content, current_file_path):
    """
    마크다운 내용의 상대경로 링크를 GitHub 절대경로로 변환
    
    Args:
        content: 마크다운 파일 내용
        current_file_path: 현재 파일의 경로 (learning_guide 기준)
    
    Returns:
        변환된 내용
    """
    
    # 현재 파일의 디렉토리
    current_dir = os.path.dirname(current_file_path)
    
    def replace_link(match):
        link_text = match.group(1)
        link_path = match.group(2)
        
        # 이미 절대 URL이면 그대로 유지
        if link_path.startswith('http://') or link_path.startswith('https://'):
            return match.group(0)
        
        # 앵커 링크면 그대로 유지
        if link_path.startswith('#'):
            return match.group(0)
        
        # 상대경로 처리
        if link_path.startswith('../'):
            # ../ 로 시작하는 경우
            resolved_path = os.path.normpath(os.path.join(current_dir, link_path))
        elif link_path.startswith('./'):
            # ./ 로 시작하는 경우
            resolved_path = os.path.normpath(os.path.join(current_dir, link_path[2:]))
        elif '/' not in link_path and link_path.endswith('.md'):
            # 같은 디렉토리의 파일
            resolved_path = os.path.join(current_dir, link_path)
        else:
            # 이미 절대경로 형태거나 특수한 경우
            resolved_path = link_path
        
        # GitHub URL 생성
        github_url = f"{GITHUB_BASE}/{resolved_path}"
        
        return f"[{link_text}]({github_url})"
    
    # 마크다운 링크 패턴: [텍스트](경로)
    pattern = r'\[([^\]]+)\]\(([^)]+)\)'
    
    converted = re.sub(pattern, replace_link, content)
    
    return converted

def process_file(file_path, base_dir):
    """
    단일 마크다운 파일 처리
    
    Args:
        file_path: 파일 전체 경로
        base_dir: learning_guide 디렉토리 경로
    """
    
    # learning_guide 기준 상대 경로
    relative_path = os.path.relpath(file_path, base_dir)
    
    print(f"Processing: {relative_path}")
    
    # 파일 읽기
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 링크 변환
    converted = convert_relative_to_github(content, relative_path)
    
    # 변경사항이 있는지 확인
    if content != converted:
        # 파일 쓰기
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(converted)
        print(f"  ✅ Updated")
        return True
    else:
        print(f"  ⏭️  No changes needed")
        return False

def main():
    """메인 함수"""
    
    # learning_guide 디렉토리
    base_dir = Path(__file__).parent
    
    print(f"Base directory: {base_dir}")
    print(f"GitHub base URL: {GITHUB_BASE}")
    print("="*70)
    
    # 처리할 마크다운 파일 찾기
    md_files = []
    
    # 모든 챕터 디렉토리
    for chapter_dir in base_dir.glob('*/'):
        if chapter_dir.is_dir() and not chapter_dir.name.startswith('.'):
            # 해당 챕터의 모든 .md 파일
            md_files.extend(chapter_dir.glob('*.md'))
    
    # 루트의 .md 파일들
    md_files.extend(base_dir.glob('*.md'))
    
    print(f"\nFound {len(md_files)} markdown files\n")
    
    # 각 파일 처리
    updated_count = 0
    for md_file in sorted(md_files):
        if process_file(md_file, base_dir):
            updated_count += 1
    
    print("\n" + "="*70)
    print(f"✅ Complete!")
    print(f"   Total files: {len(md_files)}")
    print(f"   Updated: {updated_count}")
    print(f"   Unchanged: {len(md_files) - updated_count}")

if __name__ == "__main__":
    main()
