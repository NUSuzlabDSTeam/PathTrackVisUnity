#pragma once

// 共有メモリマネージャ！  by Okuda
//  
//  仮想関数を持たない型ならＯＫ！たぶんｗ
//  ShrMemManager<Hogehoge> shr("SOME_UNIQUE_STRING") でHogehoge型用共有メモリ領域作成
//  shr.Write( hoge_instance ); でhoge_instanceを書き込み
//  shr.Read( hoge_instance ); でhoge_instanceにデータを読み込み
//  shr.LockMem(); で反対側プロセスとの排他制御のためのミューテックスを取得
//  she.UnlockMem(); でミューテックスを解放

#include <windows.h>


template <class T>
class ShrMemManager
{
	friend class ScopedLock;
private:
	HANDLE hMap; /* ファイルマッピングオブジェクトのハンドル */
	//SHARED_DATA	CurrentData, *pData; /* 共有するデータ */
	BOOL bAlreadyExists; /* 既にファイルマッピングオブジェクトが作成されているかどうかを表す真偽値 */
	//char msg[1000];
	HANDLE hMutex; /* 共有データへの排他アクセス用ミューテックスオブジェクトのハンドル */

public:
	T* pData;
	
	void LockMem();
	void UnlockMem();
public:
	ShrMemManager();
	ShrMemManager(LPCTSTR pSharePageName, LPCTSTR pMutexName=NULL);
	LRESULT Open(LPCTSTR pSharePageName, LPCTSTR pMutexName=NULL);

	~ShrMemManager(void);

	LRESULT Read(T &out);
	LRESULT Write(const T &in);

};

template<class T>
ShrMemManager<T>::ShrMemManager() : hMap(NULL), hMutex(NULL), pData(NULL)
{

}

template<class T>
ShrMemManager<T>::ShrMemManager(LPCTSTR pSharePageName, LPCTSTR pMutexPostFix) : hMap(NULL), hMutex(NULL), pData(NULL)
{
	Open(pSharePageName, pMutexPostFix);
}

template<class T>
LRESULT ShrMemManager<T>::Open(LPCTSTR pSharePageName, LPCTSTR pMutexName)
{
	std::wstring str(pSharePageName);
	if(pMutexName != NULL)
	{
		str = pMutexName;
	}else{
		str = str + L"_MUTEX";
	}
	
	/* 排他制御用ミューテックスオブジェクト作成 */
	hMutex = CreateMutex(NULL, FALSE, str.c_str());

	/* ファイルマッピングオブジェクトの作成 */
	hMap = CreateFileMapping(
		INVALID_HANDLE_VALUE,
		NULL,
		PAGE_READWRITE | SEC_COMMIT,
		0, sizeof(T), 
		pSharePageName);
	if(hMap==NULL)
	{
		str = str + L"共有メモリのオープンに失敗しました！！ 共有ページ名:";
		MessageBox(NULL, str.c_str(), L"Error!!", MB_OK);
		return S_FALSE;
	}

	/* 既に作成されていた? */
	bAlreadyExists = (GetLastError() == ERROR_ALREADY_EXISTS);

	/* ビューの作成 */
	pData = (T *)MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(T));
	return S_OK;
}

template <class T>
ShrMemManager<T>::~ShrMemManager(void)
{
	/* 後処理 */
	CloseHandle(hMutex);
	UnmapViewOfFile(pData);
	CloseHandle(hMap);
}


template <class T>
LRESULT ShrMemManager<T>::Read(T &out)
{
	if(hMap==NULL)
	{
		OutputDebugString(L"共有メモリが開けていません＠読み込み");
		return S_FALSE;
	}
	/* 共有メモリに前ページ読み */
	LockMem();//WaitForSingleObject(hMutex, INFINITE);
	out = *pData;
	UnlockMem();//ReleaseMutex(hMutex);

	return S_OK;
};

template <class T>
LRESULT ShrMemManager<T>::Write(const T &in)
{
	if(hMap==NULL)
	{
		OutputDebugString(L"共有メモリが開けていません＠書き込み");
		return S_FALSE;
	}
	/* 共有メモリに前ページ書き */

	LockMem();//WaitForSingleObject(hMutex, INFINITE);
	*pData = in;
	UnlockMem();//ReleaseMutex(hMutex);

	return S_OK;
};

template <class T>
void ShrMemManager<T>::LockMem()
{
	WaitForSingleObject(hMutex, INFINITE);
}

template <class T>
void ShrMemManager<T>::UnlockMem()
{
	ReleaseMutex(hMutex);
}
